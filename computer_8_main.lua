--[[============================================================================
  FUSEE Creat Aeronautics  --  ORDINATEUR PRINCIPAL ("computer_8")
  ---------------------------------------------------------------------------
  Pilote tout par thrust vectoring + modulation de puissance.

  Orientation par GPS (au lieu d'un gimbal sensor) :
    - computer_8 (ici)  : sa propre position P8 via gps.locate()
    - computer_5        : place a X+3 de P8 -> envoie sa position P5 (role "X")
    - computer_9        : place a Y+3 de P8 -> envoie sa position P9 (role "Y")
    On reconstruit l'orientation par produit vectoriel :
        vX = P5 - P8   (axe X du corps de la fusee, dans le monde)
        vY = P9 - P8   (axe Y du corps = "haut" de la fusee)
        up = normalize(vY)                 -> direction "haut" reelle
        vZ = vX x vY                        -> axe Z du corps (produit vectoriel)
    L'inclinaison est la part horizontale de `up`, projetee sur les axes corps.

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
local ARM_X = 3   -- computer_5 a X+3
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
local MAX_TILT = 0.35

-- Sens de correction. A basculer si un axe DIVERGE (voir calibration dans README).
local ATT_INVERT_X = false   -- attitude, axe X du corps
local ATT_INVERT_Z = false   -- attitude, axe Z du corps
local POS_INVERT_X = false   -- position, axe X du corps
local POS_INVERT_Z = false   -- position, axe Z du corps

-- ---- Reglages fins ----------------------------------------------------------
local DT          = 0.1    -- periode de boucle (s). 0.05 = 1 tick MC.
local FILTER      = 0.5    -- lissage des positions 0..1 (1 = brut, bas = lisse).
local GPS_TIMEOUT = 0.4    -- timeout gps.locate() (s).
local RX_TIMEOUT  = 1.0    -- au-dela, une position capteur est jugee "perimee".
local I_LIMIT     = 15     -- borne anti-windup des termes integraux.
local TILT_DEADB  = 0.01   -- zone morte sur l'inclinaison (anti-jitter).

-- Protocoles rednet (doivent matcher les autres ordis).
local PROTO_SENSOR = "rkt_sensor"   -- <- capteurs (computer_5 / computer_9)
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

-- Modem wireless pour rednet (le meme sert au gps.locate()).
local modem = peripheral.find("modem", function(_, m) return m.isWireless() end)
if not modem then error("Aucun modem wireless (necessaire GPS + rednet).", 0) end
rednet.open(peripheral.getName(modem))

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
  P5 = nil, P9 = nil,     -- dernieres positions recues des bras
  t5 = -1e9, t9 = -1e9,   -- horodatage de reception (os.clock)
  target = nil,           -- { x =, y =, z = }  (y = altitude visee)
  armed = false,
  legs = false,
}

-- Positions lissees (EMA) pour reduire le bruit / la granularite GPS.
local filt = { P8 = nil, P5 = nil, P9 = nil }
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
      state.P5, state.t5 = msg.pos, os.clock()
    elseif msg.role == "Y" then
      state.P9, state.t9 = msg.pos, os.clock()
    end

  elseif proto == PROTO_CMD then
    if msg.target ~= nil then state.target = msg.target end
    if msg.armed  ~= nil then state.armed  = msg.armed  end
    if msg.legs   ~= nil then state.legs   = msg.legs; setLegs(state.legs) end
    if msg.gains then
      if msg.gains.att then pidAttX:setGains(msg.gains.att); pidAttZ:setGains(msg.gains.att) end
      if msg.gains.pos then pidPosX:setGains(msg.gains.pos); pidPosZ:setGains(msg.gains.pos) end
      if msg.gains.alt then pidAlt:setGains(msg.gains.alt) end
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
  local freshX = (now - state.t5) <= RX_TIMEOUT and state.P5 ~= nil
  local freshY = (now - state.t9) <= RX_TIMEOUT and state.P9 ~= nil
  local P5 = ema("P5", freshX and state.P5 or nil)
  local P9 = ema("P9", freshY and state.P9 or nil)

  local haveOrient = P8 and P5 and P9 and freshX and freshY
  tlm.ok = haveOrient and (P8raw ~= nil)

  -- Securite : desarme, ou pas de position -> tout couper.
  if not state.armed or not P8 then
    allOff()
    tlm.P8, tlm.power, tlm.armed = P8, 0, state.armed
    return
  end

  -- 3) Orientation par produit vectoriel
  local tiltX, tiltZ = 0, 0
  local up, bxh, bzh
  if haveOrient then
    local vX = vsub(P5, P8)         -- axe X du corps
    local vY = vsub(P9, P8)         -- axe Y du corps (haut)
    up = vnorm(vY)
    local vZ = vcross(vX, vY)       -- axe Z du corps (produit vectoriel)

    -- reperes horizontaux des axes corps, dans le monde
    bxh = hnorm(vX.x, vX.z)
    bzh = hnorm(vZ.x, vZ.z)

    if up and bxh and bzh then
      -- part horizontale du "haut" = inclinaison, projetee sur les axes corps
      tiltX = up.x * bxh.x + up.z * bxh.z
      tiltZ = up.x * bzh.x + up.z * bzh.z
      if math.abs(tiltX) <= TILT_DEADB then tiltX = 0 end
      if math.abs(tiltZ) <= TILT_DEADB then tiltZ = 0 end
    else
      up, bxh, bzh = nil, nil, nil
    end
  end

  -- 4) Boucle de POSITION (externe) -> consigne d'inclinaison
  local setTiltX, setTiltZ = 0, 0
  if state.target and bxh and bzh then
    -- ecart horizontal monde -> repere corps
    local ex = state.target.x - P8.x
    local ez = state.target.z - P8.z
    local errBX = ex * bxh.x + ez * bxh.z   -- ecart le long de X corps
    local errBZ = ex * bzh.x + ez * bzh.z   -- ecart le long de Z corps

    local uX = pidPosX:step(errBX, dt)
    local uZ = pidPosZ:step(errBZ, dt)
    if POS_INVERT_X then uX = -uX end
    if POS_INVERT_Z then uZ = -uZ end
    setTiltX = clamp(uX, -MAX_TILT, MAX_TILT)
    setTiltZ = clamp(uZ, -MAX_TILT, MAX_TILT)
  end

  -- 5) Boucle d'ATTITUDE (interne) -> orientation thruster
  if up and bxh and bzh then
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

  -- 7) Telemetrie
  tlm.P8, tlm.up = P8, up
  tlm.tiltX, tlm.tiltZ = tiltX, tiltZ
  tlm.setTiltX, tlm.setTiltZ = setTiltX, setTiltZ
  tlm.power, tlm.armed, tlm.legs = power, state.armed, state.legs
  tlm.target = state.target
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
  line(4, string.format("Tilt X=%+.3f Z=%+.3f", tlm.tiltX or 0, tlm.tiltZ or 0))
  line(5, string.format("Csg  X=%+.3f Z=%+.3f", tlm.setTiltX or 0, tlm.setTiltZ or 0))
  line(6, string.format("Puissance %2d/15   Trains:%s",
        tlm.power or 0, tlm.legs and "SORTIS" or "rentres"))
  if not isMonitor then
    line(7, "")
    line(8, "Ctrl+T pour arreter (coupe tout).")
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

local ok, err = pcall(run)
allOff()
term.setCursorPos(1, 1)
if not ok and err then
  print("Erreur : " .. tostring(err))
else
  print("Arret. Thruster coupe.")
end
