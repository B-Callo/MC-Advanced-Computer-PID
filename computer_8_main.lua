--[[============================================================================
  FUSEE Creat Aeronautics  --  ORDINATEUR PRINCIPAL ("computer_8")
  ---------------------------------------------------------------------------
  Pilote tout par thrust vectoring + modulation de puissance.

  Orientation par GPS (au lieu d'un gimbal sensor) :
    - computer_8 (ici)  : sa propre position P8 via gps.locate()
    - computer_6        : bras HORIZONTAL 1 -> envoie sa position P6 (role "X")
    - computer_9        : bras HORIZONTAL 2, perpendiculaire -> P9 (role "Y")
    Les DEUX bras sont HORIZONTAUX et perpendiculaires. Le "haut" de la fusee
    est la NORMALE a leur plan = leur PRODUIT VECTORIEL :
        vA = P6 - P8   vB = P9 - P8         (les deux bras, dans le repere monde)
        ey = normalize(vA x vB)             -> axe haut du corps (remis vers le haut)
        ex = normalize(vA - (vA.ey) ey)     -> axe X corps, orthogonal a ey
        ez = ex x ey                        -> axe Z corps
    Inclinaison = verticale du monde (0,1,0) vue dans ce repere :
        tiltX = ex.y   tiltZ = ez.y   (0 si la fusee est droite).

  Cascade de 3 PID :
    1. ATTITUDE (interne) : suit une consigne d'inclinaison -> oriente le thruster
       (4 signaux redstone sur redstone_relay_7).
    2. POSITION (externe)  : ecart X/Z monde vs cible -> consigne d'inclinaison pour
       la boucle 1 (la fusee s'incline vers la cible, transporte, puis se redresse).
    3. ALTITUDE            : ecart Y vs cible -> puissance du thruster
       (front de redstone_relay_8).

  Trains d'atterrissage : back de redstone_relay_8 (on/off).
  Consignes et gains recus en direct depuis le terminal ("computer_7") par rednet.
  Affichage sur l'ecran "screen_1" (monitor 1x3) + resume sur le terminal local.

  ==> Tout se configure dans la section CONFIG ci-dessous.
============================================================================]]--

--============================ CONFIG (a remplir) ============================--

-- Peripheriques (noms tels qu'affiches par peripheral.getNames()).
local RELAY_ORIENT = "redstone_relay_7"  -- orientation du vector thruster (4 dir)
local RELAY_POWER  = "redstone_relay_8"  -- puissance (front) + trains (back)
local SCREEN       = "screen_1"          -- monitor d'affichage (optionnel)

-- ORIENTATION du thruster (redstone_relay_7). Une face = une direction de poussee.
--   vers X = front, vers -X = back, vers Z = right, vers -Z = left.
local ORIENT = {
  posX = "front",   -- pousse vers +X (corps)
  negX = "back",    -- pousse vers -X
  posZ = "right",   -- pousse vers +Z
  negZ = "left",    -- pousse vers -Z
}

-- PUISSANCE + TRAINS (redstone_relay_8).
local POWER_SIDE = "front"   -- puissance du thruster (0-15)
local LEGS_SIDE  = "back"    -- trains d'atterrissage (0 = rentres, 15 = sortis)

-- Bras de levier des ordinateurs capteurs (blocs). Sert de repere, on renormalise
-- de toute facon ; a titre indicatif / verif de coherence.
local ARM_X = 3   -- computer_6 a X+3
local ARM_Y = 3   -- computer_9 a Y+3

-- ---- Gains PID par defaut (le terminal peut les remplacer en direct) --------
-- 1) ATTITUDE (interne) : inclinaison -> orientation thruster.
local ATT_GAINS = { kp = 6.0, ki = 0.0, kd = 3.0 }
-- 2) POSITION (externe) : ecart horizontal -> consigne d'inclinaison. PD conseille.
local POS_GAINS = { kp = 0.35, ki = 0.0, kd = 0.80 }
-- 3) ALTITUDE : ecart de hauteur -> puissance.
local ALT_GAINS = { kp = 2.0, ki = 0.15, kd = 1.5 }

-- Puissance d'equilibre (hover) ajoutee a la sortie du PID d'altitude. A regler
-- pour que la fusee tienne son altitude quand l'ecart est nul (~ poids / poussee).
local BASE_POWER = 8

-- Consigne d'inclinaison maxi demandee par la boucle de position (unites "tilt",
-- ou 1.0 = fusee couchee a l'horizontale). Limite l'agressivite du deplacement.
local MAX_TILT = 0.707   -- = sin(45 deg) : inclinaison maxi demandee ~45 degres

-- Distance horizontale (blocs) au-dela de laquelle l'ecart est sature : la fusee
-- demande simplement le lean maxi vers la cible, avec un ecart constant (pas de
-- terme derive qui s'emballe -> plus de clignotement de la consigne). En-deca, le
-- PID de position agit normalement pour freiner et s'arreter en douceur.
local APPROACH_DIST = 20

-- Sens de correction. A basculer si un axe DIVERGE (voir calibration dans README).
local ATT_INVERT_X = false   -- attitude, axe X du corps
local ATT_INVERT_Z = false   -- attitude, axe Z du corps
local POS_INVERT_X = true    -- position, axe X du corps (sens naturellement inverse)
local POS_INVERT_Z = true    -- position, axe Z du corps (sens naturellement inverse)

-- ---- Reglages fins ----------------------------------------------------------
local DT          = 0.1    -- periode de boucle (s). 0.05 = 1 tick MC.
local FILTER      = 0.5    -- lissage des positions 0..1 (1 = brut, bas = lisse).
local GPS_TIMEOUT = 0.4    -- timeout gps.locate() (s).
local RX_TIMEOUT  = 1.0    -- au-dela, une position capteur est jugee "perimee".
local I_LIMIT     = 15     -- borne anti-windup des termes integraux.
local TILT_DEADB  = 0.01   -- zone morte sur l'inclinaison (anti-jitter).

-- Protocoles rednet (doivent matcher les autres ordis).
local PROTO_SENSOR = "rkt_sensor"   -- <- capteurs (computer_6 / computer_9)
local PROTO_CMD    = "rkt_cmd"      -- <- terminal (computer_7)
local PROTO_TLM    = "rkt_tlm"      -- -> telemetrie diffusee

--========================= FIN DE LA CONFIG ================================--


--------------------------------------------------------------------------------
-- Helpers vecteurs / maths
--------------------------------------------------------------------------------
local function clamp(x, lo, hi)
  if x < lo then return lo elseif x > hi then return hi else return x end
end

local function vsub(a, b) return { x = a.x - b.x, y = a.y - b.y, z = a.z - b.z } end
local function vlen(a)    return math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z) end

local function vnorm(a)
  local l = vlen(a)
  if l < 1e-9 then return nil end
  return { x = a.x / l, y = a.y / l, z = a.z / l }
end

-- produit vectoriel a x b
local function vcross(a, b)
  return {
    x = a.y * b.z - a.z * b.y,
    y = a.z * b.x - a.x * b.z,
    z = a.x * b.y - a.y * b.x,
  }
end

-- normalise le vecteur horizontal (x,z) ; renvoie {x,z} unitaire ou nil
local function hnorm(vx, vz)
  local l = math.sqrt(vx*vx + vz*vz)
  if l < 1e-9 then return nil end
  return { x = vx / l, z = vz / l }
end

--------------------------------------------------------------------------------
-- PID generique (agit directement sur l'erreur)
--------------------------------------------------------------------------------
local function newPID(g)
  return {
    kp = g.kp, ki = g.ki or 0, kd = g.kd or 0,
    integral = 0, prev = 0,
    step = function(self, err, dt)
      self.integral = clamp(self.integral + err * dt, -I_LIMIT, I_LIMIT)
      local d = (err - self.prev) / dt
      self.prev = err
      return self.kp * err + self.ki * self.integral + self.kd * d
    end,
    setGains = function(self, g2)
      self.kp, self.ki, self.kd = g2.kp, g2.ki or 0, g2.kd or 0
    end,
    reset = function(self) self.integral = 0; self.prev = 0 end,
    resetI = function(self) self.integral = 0 end,  -- remet a zero l'integral seul
  }
end

--------------------------------------------------------------------------------
-- Peripheriques
--------------------------------------------------------------------------------
local relayOrient = peripheral.wrap(RELAY_ORIENT)
   or error("Relay orientation introuvable : " .. RELAY_ORIENT, 0)
local relayPower  = peripheral.wrap(RELAY_POWER)
   or error("Relay puissance introuvable : " .. RELAY_POWER, 0)
local screen      = peripheral.wrap(SCREEN)   -- peut etre nil (optionnel)

-- Ouvre rednet sur TOUS les modems (filaire pour la comm interne avec les autres
-- ordis). Le modem WIRELESS reste indispensable pour gps.locate() (pas de GPS en filaire).
local opened = 0
for _, name in ipairs(peripheral.getNames()) do
  if peripheral.getType(name) == "modem" then
    rednet.open(name)
    opened = opened + 1
  end
end
if opened == 0 then error("Aucun modem : rednet impossible.", 0) end

--------------------------------------------------------------------------------
-- Sorties
--------------------------------------------------------------------------------
local function roundRS(x) return clamp(math.floor(math.abs(x) + 0.5), 0, 15) end

-- Applique une commande sur une paire de faces opposees du relay d'orientation.
local function driveAxis(u, posSide, negSide)
  if u >= 0 then
    relayOrient.setAnalogOutput(posSide, roundRS(u))
    relayOrient.setAnalogOutput(negSide, 0)
  else
    relayOrient.setAnalogOutput(posSide, 0)
    relayOrient.setAnalogOutput(negSide, roundRS(u))
  end
end

local function orientOff()
  relayOrient.setAnalogOutput(ORIENT.posX, 0)
  relayOrient.setAnalogOutput(ORIENT.negX, 0)
  relayOrient.setAnalogOutput(ORIENT.posZ, 0)
  relayOrient.setAnalogOutput(ORIENT.negZ, 0)
end

local function setPower(p) relayPower.setAnalogOutput(POWER_SIDE, roundRS(p)) end
local function setLegs(on) relayPower.setAnalogOutput(LEGS_SIDE, on and 15 or 0) end

local function allOff()
  orientOff()
  setPower(0)
  -- on laisse les trains dans leur etat courant (securite au sol)
end

--------------------------------------------------------------------------------
-- Etat partage (mis a jour par les messages rednet)
--------------------------------------------------------------------------------
local state = {
  P6 = nil, P9 = nil,     -- dernieres positions recues des bras
  t6 = -1e9, t9 = -1e9,   -- horodatage de reception (os.clock)
  target = nil,           -- { x =, y =, z = }  (y = altitude visee)
  armed = false,
  legs = false,
}

-- Positions lissees (EMA) pour reduire le bruit / la granularite GPS.
local filt = { P8 = nil, P6 = nil, P9 = nil }
local function ema(key, p)
  if not p then return filt[key] end
  local f = filt[key]
  if not f then filt[key] = { x = p.x, y = p.y, z = p.z }
  else
    f.x = FILTER * p.x + (1 - FILTER) * f.x
    f.y = FILTER * p.y + (1 - FILTER) * f.y
    f.z = FILTER * p.z + (1 - FILTER) * f.z
  end
  return filt[key]
end

--------------------------------------------------------------------------------
-- Traitement des messages rednet
--------------------------------------------------------------------------------
local function handleMessage(msg, proto)
  if type(msg) ~= "table" then return end

  if proto == PROTO_SENSOR and msg.pos then
    if msg.role == "X" then
      state.P6, state.t6 = msg.pos, os.clock()
    elseif msg.role == "Y" then
      state.P9, state.t9 = msg.pos, os.clock()
    end

  elseif proto == PROTO_CMD then
    if msg.target ~= nil then state.target = msg.target; saveCfg() end
    if msg.armed  ~= nil then state.armed  = msg.armed  end
    if msg.legs   ~= nil then state.legs   = msg.legs; setLegs(state.legs) end
    if msg.gains then
      if msg.gains.att then gains.att = msg.gains.att end
      if msg.gains.pos then gains.pos = msg.gains.pos end
      if msg.gains.alt then gains.alt = msg.gains.alt end
      applyGains()
      saveCfg()   -- persiste gains + cible : survit au reboot
    end
    if msg.resetI then
      pidAttX:resetI(); pidAttZ:resetI()
      pidPosX:resetI(); pidPosZ:resetI()
      pidAlt:resetI()
    end
  end
end

--------------------------------------------------------------------------------
-- PID (declares en amont pour handleMessage)
--------------------------------------------------------------------------------
pidAttX = newPID(ATT_GAINS)
pidAttZ = newPID(ATT_GAINS)
pidPosX = newPID(POS_GAINS)
pidPosZ = newPID(POS_GAINS)
pidAlt  = newPID(ALT_GAINS)

--------------------------------------------------------------------------------
-- Persistance sur disque (survit au reboot du principal) : gains + cible.
--   Fichier rocket.cfg (serialise). Charge au boot, reecrit a chaque changement.
--   Globaux car reference par handleMessage (defini plus haut).
--------------------------------------------------------------------------------
local CFG_FILE = "rocket.cfg"
gains = { att = ATT_GAINS, pos = POS_GAINS, alt = ALT_GAINS }

function applyGains()
  pidAttX:setGains(gains.att); pidAttZ:setGains(gains.att)
  pidPosX:setGains(gains.pos); pidPosZ:setGains(gains.pos)
  pidAlt:setGains(gains.alt)
end

-- Sauvegarde gains + cible. On NE persiste PAS 'armed' (securite : au reboot la
-- fusee reste desarmee tant qu'on ne l'a pas rearmee explicitement).
function saveCfg()
  local f = fs.open(CFG_FILE, "w")
  if f then
    f.write(textutils.serialize({ gains = gains, target = state.target }))
    f.close()
  end
end

function loadCfg()
  if not fs.exists(CFG_FILE) then return end
  local f = fs.open(CFG_FILE, "r")
  if not f then return end
  local data = textutils.unserialize(f.readAll() or "")
  f.close()
  if type(data) ~= "table" then return end
  if type(data.gains) == "table" then
    if type(data.gains.att) == "table" then gains.att = data.gains.att end
    if type(data.gains.pos) == "table" then gains.pos = data.gains.pos end
    if type(data.gains.alt) == "table" then gains.alt = data.gains.alt end
  end
  if type(data.target) == "table" then state.target = data.target end
end

--------------------------------------------------------------------------------
-- Un pas de controle
--------------------------------------------------------------------------------
local last = os.clock()
local tlm = { ok = false }   -- derniere telemetrie (pour affichage + diffusion)

local function controlStep()
  local now = os.clock()
  local dt = now - last
  if dt <= 0 then dt = DT end
  last = now

  -- 1) Position propre P8 par GPS
  local x, y, z = gps.locate(GPS_TIMEOUT)
  local P8raw = x and { x = x, y = y, z = z } or nil
  local P8 = ema("P8", P8raw)

  -- 2) Fraicheur des capteurs de bras
  local freshX = (now - state.t6) <= RX_TIMEOUT and state.P6 ~= nil
  local freshY = (now - state.t9) <= RX_TIMEOUT and state.P9 ~= nil
  local P6 = ema("P6", freshX and state.P6 or nil)
  local P9 = ema("P9", freshY and state.P9 or nil)

  local haveOrient = P8 and P6 and P9 and freshX and freshY
  tlm.ok = haveOrient and (P8raw ~= nil)

  -- 3) Orientation : les DEUX bras sont HORIZONTAUX et perpendiculaires
  --    (computer_6 sur un axe du corps, computer_9 sur l'axe perpendiculaire).
  --    Le "haut" de la fusee est donc la NORMALE au plan des deux bras, soit
  --    leur PRODUIT VECTORIEL. On en tire un repere corps orthonormalise :
  --      vA = P6 - P8   (bras 1)          vB = P9 - P8   (bras 2, perpendiculaire)
  --      ey = normalize(vA x vB)          (retourne vers le haut si besoin)
  --      ex = normalize(vA - (vA.ey) ey)  (axe corps dans le plan, perp. a ey)
  --      ez = ex x ey
  --    Inclinaison = verticale du monde (0,1,0) vue dans ce repere :
  --      tiltX = ex.y   tiltZ = ez.y   (0 si la fusee est droite).
  local tiltX, tiltZ = 0, 0
  local ex, ey, ez          -- axes du corps (unitaires) dans le repere monde
  local exh, ezh            -- directions horizontales de ces axes (pour la position)
  local lenA, lenB = 0, 0
  if haveOrient then
    local vA = vsub(P6, P8)         -- bras 1 (doit mesurer ~3 blocs)
    local vB = vsub(P9, P8)         -- bras 2 (doit mesurer ~3 blocs)
    lenA, lenB = vlen(vA), vlen(vB)

    ey = vnorm(vcross(vA, vB))                       -- normale au plan des bras
    if ey and ey.y < 0 then                          -- s'assure qu'elle pointe en haut
      ey = { x = -ey.x, y = -ey.y, z = -ey.z }
    end
    if ey then
      local d = vA.x*ey.x + vA.y*ey.y + vA.z*ey.z    -- projection de vA sur ey
      ex = vnorm({ x = vA.x - d*ey.x, y = vA.y - d*ey.y, z = vA.z - d*ey.z })
    end
    if ex and ey then ez = vnorm(vcross(ex, ey)) end

    if ex and ey and ez then
      tiltX = ex.y
      tiltZ = ez.y
      if math.abs(tiltX) <= TILT_DEADB then tiltX = 0 end
      if math.abs(tiltZ) <= TILT_DEADB then tiltZ = 0 end
      exh = hnorm(ex.x, ex.z)       -- direction horizontale de l'axe X corps
      ezh = hnorm(ez.x, ez.z)       -- direction horizontale de l'axe Z corps
    else
      ex, ey, ez = nil, nil, nil
    end
  end

  -- Telemetrie d'orientation : calculee EN PERMANENCE (meme desarmee), pour
  -- pouvoir observer/calibrer les tilts fusee posee et a la main.
  tlm.P8, tlm.P6, tlm.P9 = P8, P6, P9
  tlm.lenX, tlm.lenY = lenA, lenB
  tlm.ageX, tlm.ageY = now - state.t6, now - state.t9
  tlm.up = ey            -- axe Y du corps (haut) dans le monde ; droit = (0,1,0)
  tlm.tiltX, tlm.tiltZ = tiltX, tiltZ
  tlm.target, tlm.armed, tlm.legs = state.target, state.armed, state.legs

  -- Securite : desarme, ou pas de position -> couper les actionneurs (mais on a
  -- deja calcule et remonte l'orientation ci-dessus).
  if not state.armed or not P8 then
    allOff()
    tlm.power = 0
    tlm.setTiltX, tlm.setTiltZ = 0, 0
    return
  end

  -- 4) Boucle de POSITION (externe) -> consigne d'inclinaison
  local setTiltX, setTiltZ = 0, 0
  if state.target and exh and ezh then
    -- ecart horizontal monde -> projete sur les axes horizontaux du corps
    local ax = state.target.x - P8.x
    local az = state.target.z - P8.z
    local errBX = ax * exh.x + az * exh.z   -- ecart le long de X corps
    local errBZ = ax * ezh.x + az * ezh.z   -- ecart le long de Z corps

    -- Sature la NORME de l'ecart a APPROACH_DIST : sur une cible lointaine on
    -- demande juste "lean maxi vers la cible" avec un ecart CONSTANT -> le terme
    -- derive retombe a ~0 (fini le clignotement bang-bang +/-MAX_TILT). En-deca,
    -- l'ecart est reel et le PID agit normalement pour un arret en douceur.
    local emag = math.sqrt(errBX * errBX + errBZ * errBZ)
    if emag > APPROACH_DIST and emag > 0 then
      local s = APPROACH_DIST / emag
      errBX, errBZ = errBX * s, errBZ * s
    end

    local uX = pidPosX:step(errBX, dt)
    local uZ = pidPosZ:step(errBZ, dt)
    if POS_INVERT_X then uX = -uX end
    if POS_INVERT_Z then uZ = -uZ end
    setTiltX = clamp(uX, -MAX_TILT, MAX_TILT)
    setTiltZ = clamp(uZ, -MAX_TILT, MAX_TILT)
  end

  -- 5) Boucle d'ATTITUDE (interne) -> orientation thruster
  if ex and ey and ez then
    local cmdX = pidAttX:step(setTiltX - tiltX, dt)
    local cmdZ = pidAttZ:step(setTiltZ - tiltZ, dt)
    if ATT_INVERT_X then cmdX = -cmdX end
    if ATT_INVERT_Z then cmdZ = -cmdZ end
    driveAxis(clamp(cmdX, -15, 15), ORIENT.posX, ORIENT.negX)
    driveAxis(clamp(cmdZ, -15, 15), ORIENT.posZ, ORIENT.negZ)
  else
    -- orientation inconnue : thruster droit (pas de vectoring), on garde le hover
    orientOff()
  end

  -- 6) Boucle d'ALTITUDE -> puissance
  local power = BASE_POWER
  if state.target and state.target.y then
    power = BASE_POWER + pidAlt:step(state.target.y - P8.y, dt)
  end
  power = clamp(power, 0, 15)
  setPower(power)

  -- 7) Telemetrie (le reste a deja ete remonte plus haut)
  tlm.setTiltX, tlm.setTiltZ = setTiltX, setTiltZ
  tlm.power, tlm.legs = power, state.legs
end

--------------------------------------------------------------------------------
-- Affichage (ecran monitor + terminal local)
--------------------------------------------------------------------------------
local function fmtVec(p)
  if not p then return "   ?    ?    ?" end
  return string.format("%6.1f %6.1f %6.1f", p.x, p.y, p.z)
end

local function drawTo(out, isMonitor)
  out.clear()
  out.setCursorPos(1, 1)
  local w = out.getSize and select(1, out.getSize()) or 51
  local function line(n, s) out.setCursorPos(1, n); out.write(s) end

  line(1, "== FUSEE " .. (tlm.armed and "[ARMEE]" or "[SECU] ")
        .. (tlm.ok and "" or " capteurs?"))
  line(2, "Pos  " .. fmtVec(tlm.P8))
  line(3, "Cible" .. fmtVec(tlm.target))
  local u = tlm.up
  line(4, "AxeY " .. (u and string.format("%+.2f %+.2f %+.2f", u.x, u.y, u.z)
        or "  ?    ?    ?") .. " (droit=0,1,0)")
  line(5, string.format("Tilt X=%+.3f Z=%+.3f", tlm.tiltX or 0, tlm.tiltZ or 0))
  line(6, string.format("Csg  X=%+.3f Z=%+.3f", tlm.setTiltX or 0, tlm.setTiltZ or 0))
  line(7, string.format("Bras |X|=%.1f |Y|=%.1f  ageXY=%.1f/%.1fs",
        tlm.lenX or 0, tlm.lenY or 0, tlm.ageX or 9, tlm.ageY or 9))
  line(8, string.format("Puissance %2d/15   Trains:%s",
        tlm.power or 0, tlm.legs and "SORTIS" or "rentres"))
  if not isMonitor then
    line(9, "")
    line(10, "Ctrl+T pour arreter (coupe tout).")
  end
end

local function refreshScreens()
  if screen then
    pcall(function() screen.setTextScale(0.5) end)
    pcall(drawTo, screen, true)
  end
  pcall(drawTo, term, false)
end

--------------------------------------------------------------------------------
-- Boucle principale : evenementielle (rednet + timer)
--------------------------------------------------------------------------------
local function run()
  loadCfg()      -- recharge gains + cible sauvegardes (sinon valeurs par defaut)
  applyGains()
  allOff()
  setLegs(state.legs)
  local timer = os.startTimer(DT)
  while true do
    local ev = { os.pullEvent() }
    if ev[1] == "rednet_message" then
      handleMessage(ev[3], ev[4])
    elseif ev[1] == "timer" and ev[2] == timer then
      controlStep()
      refreshScreens()
      rednet.broadcast(tlm, PROTO_TLM)
      timer = os.startTimer(DT)
    end
  end
end

-- Ecoute un ordre de mise a jour (rednet). En sortant, on arrete la boucle de
-- controle (waitForAny) AVANT de mettre a jour, pour ne pas re-appliquer de
-- poussee pendant le compte a rebours de update.lua.
local doUpdate = false
local function updateListener()
  while true do
    local _, msg = rednet.receive("rkt_update")
    if msg == "update" then doUpdate = true; return end
  end
end

local ok, err = pcall(parallel.waitForAny, run, updateListener)
allOff()                          -- coupe poussee + orientation
term.setCursorPos(1, 1)
if doUpdate then
  print("Mise a jour recue...")
  shell.run("update")             -- telecharge + reboot (si update.lua present)
elseif not ok and err then
  print("Erreur : " .. tostring(err))
else
  print("Arret. Thruster coupe.")
end
