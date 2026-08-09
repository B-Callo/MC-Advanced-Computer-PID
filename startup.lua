--[[============================================================================
  PID d'attitude pour fusee Creat Aeronautics  --  CC:Tweaked Advanced Computer
  ---------------------------------------------------------------------------
  Objectif : garder la fusee droite en stationnaire (hover) par thrust vectoring.

  Principe (double boucle en cascade) :
    - Le "gimbal sensor" renvoie 4 signaux redstone analogiques (0-15), un par
      direction d'inclinaison : nord, sud, est, ouest.
    - On calcule l'inclinaison nette sur 2 axes :
          tangage (pitch) = nord - sud
          roulis  (roll)  = est  - ouest
    - BOUCLE INTERNE (attitude) : un PID par axe suit une consigne d'inclinaison.
    - BOUCLE EXTERNE (position) : la "navigation table" (relay nav) donne l'offset
      horizontal par rapport a l'ancre. Un PD par axe transforme cet offset en
      consigne d'inclinaison -> la fusee s'incline vers l'ancre pour y revenir,
      puis se redresse. Desactivable (NAV_ENABLED) pour rester en simple hover.
    - La sortie du PID interne oriente le "vector thruster" via 4 signaux redstone.

  ==> Tout se configure dans la section CONFIG juste en dessous.
============================================================================]]--

--============================ CONFIG (a remplir) ============================--

-- Nom du peripheral redstone relay tel qu'affiche par CC.
-- Mets "computer" pour utiliser directement les 6 faces de l'ordinateur.
local RELAY_IN = "redstone_relay_2"
local RELAY_OUT = "redstone_relay_1"
local RELAY_TUNE = "redstone_relay_3"
local RELAY_NAV = "redstone_relay_4"

-- ENTREES : gimbal sensor. Une face par direction d'inclinaison.
-- device = RELAY (le relay) ou "computer" (face de l'ordi) ; side = face redstone.
local INPUTS = {
  north = { device = RELAY_IN, side = "front"    },
  south = { device = RELAY_IN, side = "back" },
  east  = { device = RELAY_IN, side = "left"   },
  west  = { device = RELAY_IN, side = "right"  },
}

-- SORTIES : orientation du vector thruster. Une face par direction de poussee.
local OUTPUTS = {
  north = { device = RELAY_OUT, side = "front" },
  south = { device = RELAY_OUT, side = "back"  },
  east  = { device = RELAY_OUT, side = "left"  },
  west  = { device = RELAY_OUT, side = "right" },
}

-- REGLAGE LIVE (relay de tune) : 3 signaux redstone 0-15 lus pour ajuster les
-- gains sans rebooter. Chaque face code un gain, remappe de 0..15 vers 0..max.
local TUNE = {
  kp = { device = RELAY_TUNE, side = "front" },
  ki = { device = RELAY_TUNE, side = "back"  },
  kd = { device = RELAY_TUNE, side = "right"  },
}

-- Active la lecture des gains depuis le relay de tune.
-- false = on utilise les valeurs fixes de GAINS ci-dessous.
local TUNE_ENABLED = true

-- Valeur de gain quand la face redstone est a 15 (plein). 0 -> gain nul.
-- Ex: kp = signal/15 * 4.0  (donc 0..4.0 par pas de 4.0/15 ~ 0.27)
local TUNE_MAX = {
  kp = 4.0,
  ki = 1.0,
  kd = 2.0,
}

-- Gains PID par defaut (utilises si TUNE_ENABLED = false).
-- Regle KP d'abord (reaction), puis KD (amortit l'oscillation), puis KI (biais).
local GAINS = {
  kp = 1.5,   -- proportionnel : force de correction immediate
  ki = 0.20,  -- integral      : corrige les erreurs qui trainent
  kd = 0.80,  -- derive        : freine / amortit les oscillations
}

-- Sens de correction. Si un axe DIVERGE (part de plus en plus fort au lieu de
-- se stabiliser), passe le flag correspondant a true (ou inverse les cables).
local INVERT_PITCH = false   -- axe nord/sud
local INVERT_ROLL  = false   -- axe est/ouest

-- ---- BOUCLE EXTERNE : maintien au-dessus de l'ancre (navigation table) ------
-- Entrees nav : 4 directions (meme orientation que le gimbal) + distance dessous.
-- Signal FORT = PROCHE de l'ancre dans cette direction.
local NAV = {
  north = { device = RELAY_NAV, side = "front"  },
  south = { device = RELAY_NAV, side = "back"   },
  east  = { device = RELAY_NAV, side = "left"   },
  west  = { device = RELAY_NAV, side = "right"  },
  dist  = { device = RELAY_NAV, side = "bottom" },  -- 15 = sur l'ancre, 0 = loin
}

-- Active le maintien de position. false = simple hover (consigne = vertical).
local NAV_ENABLED = true

-- Gains de la boucle externe (offset horizontal -> consigne d'inclinaison).
-- PD : Kp pour revenir vers l'ancre, Kd pour amortir (evite le depassement).
local NAV_GAINS = {
  kp = 0.40,
  kd = 0.60,
}

-- Consigne d'inclinaison maximale demandee a la boucle interne (unites capteur).
-- Limite l'agressivite du retour vers l'ancre.
local NAV_MAX_TILT = 5

-- Sens du retour. Si la fusee s'ELOIGNE de l'ancre au lieu de s'en rapprocher,
-- inverse l'axe correspondant (le tilt physique va dans le mauvais sens).
local NAV_INVERT_NS = false
local NAV_INVERT_EW = false

-- Offset ignore sous ce seuil (zone morte : evite de tiller pour rien pres de l'ancre).
local NAV_DEADBAND = 1

-- Reglages fins.
local DT        = 0.05  -- periode de boucle en secondes (0.05 = 1 tick MC)
local DEADBAND  = 0.0   -- inclinaison ignoree si |erreur| <= DEADBAND (anti-jitter)
local FILTER    = 0.6   -- lissage capteur 0..1 (1 = brut, plus bas = plus lisse)
local RS_MIN    = 0     -- valeur redstone mini en sortie
local RS_MAX    = 15    -- valeur redstone maxi en sortie
local I_LIMIT   = 15    -- borne anti-windup du terme integral

--========================= FIN DE LA CONFIG ================================--


--------------------------------------------------------------------------------
-- Acces redstone (computer ou relay), avec cache des peripherals.
--------------------------------------------------------------------------------
local deviceCache = {}

local function resolveDevice(name)
  if name == nil or name == "computer" or name == "local" then
    return redstone
  end
  if deviceCache[name] then return deviceCache[name] end
  local p = peripheral.wrap(name)
  if not p then
    error("Peripheral introuvable : '" .. tostring(name)
        .. "'. Verifie RELAY dans la CONFIG (peripheral.getNames()).", 0)
  end
  deviceCache[name] = p
  return p
end

local function readAnalog(spec)
  return resolveDevice(spec.device).getAnalogInput(spec.side)
end

local function writeAnalog(spec, value)
  resolveDevice(spec.device).setAnalogOutput(spec.side, value)
end

--------------------------------------------------------------------------------
-- Helpers
--------------------------------------------------------------------------------
local function clamp(x, lo, hi)
  if x < lo then return lo elseif x > hi then return hi else return x end
end

local function roundRS(x)
  local v = math.floor(math.abs(x) + 0.5)
  return clamp(v, RS_MIN, RS_MAX)
end

--------------------------------------------------------------------------------
-- Controleur PID generique (consigne = setpoint, 0 par defaut)
--------------------------------------------------------------------------------
local function newPID(g)
  return {
    kp = g.kp, ki = g.ki or 0, kd = g.kd or 0,
    integral = 0,
    prevError = 0,
    update = function(self, measurement, setpoint, dt)
      local err = (setpoint or 0) - measurement

      -- terme integral avec anti-windup
      self.integral = clamp(self.integral + err * dt, -I_LIMIT, I_LIMIT)

      -- terme derive (sur l'erreur ; setpoint constant donc pas de kick)
      local deriv = (err - self.prevError) / dt
      self.prevError = err

      return self.kp * err + self.ki * self.integral + self.kd * deriv
    end,
    reset = function(self)
      self.integral = 0
      self.prevError = 0
    end,
  }
end

--------------------------------------------------------------------------------
-- Etat capteur filtre (EMA)
--------------------------------------------------------------------------------
local filt = {}

local function readFilteredSpec(key, spec)
  local raw = readAnalog(spec)
  filt[key] = FILTER * raw + (1 - FILTER) * (filt[key] or 0)
  return filt[key]
end

-- gimbal sensor (boucle interne) et navigation table (boucle externe)
local function readIn(key)  return readFilteredSpec("in_"  .. key, INPUTS[key]) end
local function readNav(key) return readFilteredSpec("nav_" .. key, NAV[key])    end

--------------------------------------------------------------------------------
-- Lecture live des gains depuis le relay de tune (0-15 -> 0..TUNE_MAX).
--------------------------------------------------------------------------------
local function readGains()
  return {
    kp = readAnalog(TUNE.kp) / 15 * TUNE_MAX.kp,
    ki = readAnalog(TUNE.ki) / 15 * TUNE_MAX.ki,
    kd = readAnalog(TUNE.kd) / 15 * TUNE_MAX.kd,
  }
end

--------------------------------------------------------------------------------
-- Applique une commande d'axe sur une paire de sorties opposees.
--   u > 0  -> pousse vers posDir ; u < 0 -> pousse vers negDir.
--------------------------------------------------------------------------------
local function driveAxis(u, posKey, negKey)
  if u >= 0 then
    writeAnalog(OUTPUTS[posKey], roundRS(u))
    writeAnalog(OUTPUTS[negKey], 0)
  else
    writeAnalog(OUTPUTS[posKey], 0)
    writeAnalog(OUTPUTS[negKey], roundRS(u))
  end
end

local function allOff()
  for _, spec in pairs(OUTPUTS) do
    writeAnalog(spec, 0)
  end
end

--------------------------------------------------------------------------------
-- Boucle externe : offset horizontal (nav) -> consigne d'inclinaison.
--   Retourne le tilt vise (unites capteur) pour ramener la fusee sur l'ancre.
--------------------------------------------------------------------------------
local function navTilt(navPID, posKey, negKey, invert, dt)
  -- ecart directionnel a l'ancre sur cet axe (signal fort = proche)
  local axis = readNav(posKey) - readNav(negKey)
  if math.abs(axis) <= NAV_DEADBAND then axis = 0 end
  if invert then axis = -axis end
  -- PD ramenant l'ecart a 0 ; la sortie devient la consigne d'inclinaison
  local u = navPID:update(axis, 0, dt)
  return clamp(u, -NAV_MAX_TILT, NAV_MAX_TILT)
end

--------------------------------------------------------------------------------
-- Affichage
--------------------------------------------------------------------------------
local function draw(pitch, roll, uPitch, uRoll, gains, setPitch, setRoll)
  term.clear()
  term.setCursorPos(1, 1)
  print("=== PID Attitude Fusee (Ctrl+T pour arreter) ===")
  print("")
  print(string.format(" TANGAGE (N-S) : inclin=%+.2f  csg=%+.2f  cmd=%+.2f", pitch, setPitch, uPitch))
  print(string.format(" ROULIS  (E-O) : inclin=%+.2f  csg=%+.2f  cmd=%+.2f", roll,  setRoll,  uRoll))
  print("")
  if NAV_ENABLED then
    print(string.format(" NAV ancre : dist=%2d  N%2d S%2d E%2d W%2d",
      readAnalog(NAV.dist),
      readAnalog(NAV.north), readAnalog(NAV.south),
      readAnalog(NAV.east),  readAnalog(NAV.west)))
  else
    print(" NAV : desactive (hover simple)")
  end
  print("")
  print(string.format(" Gains  Kp=%.2f  Ki=%.2f  Kd=%.2f", gains.kp, gains.ki, gains.kd))
  print(TUNE_ENABLED and " (tune LIVE via relay)" or " (gains fixes)")
end

--------------------------------------------------------------------------------
-- Boucle de controle
--------------------------------------------------------------------------------
local function run()
  local pidPitch = newPID(GAINS)
  local pidRoll  = newPID(GAINS)
  local navPitch = newPID(NAV_GAINS)   -- boucle externe axe nord/sud
  local navRoll  = newPID(NAV_GAINS)   -- boucle externe axe est/ouest

  allOff()
  local last = os.clock()

  while true do
    -- dt reel
    local now = os.clock()
    local dt = now - last
    if dt <= 0 then dt = DT end
    last = now

    -- inclinaison nette par axe (boucle interne)
    local pitch = readIn("north") - readIn("south")
    local roll  = readIn("east")  - readIn("west")

    -- deadband anti-jitter
    if math.abs(pitch) <= DEADBAND then pitch = 0 end
    if math.abs(roll)  <= DEADBAND then roll  = 0 end

    -- BOUCLE EXTERNE : consigne d'inclinaison pour rester sur l'ancre
    local setPitch, setRoll = 0, 0
    if NAV_ENABLED then
      setPitch = navTilt(navPitch, "north", "south", NAV_INVERT_NS, dt)
      setRoll  = navTilt(navRoll,  "east",  "west",  NAV_INVERT_EW, dt)
    end

    -- gains internes : live depuis le relay de tune, ou valeurs fixes
    local gains = TUNE_ENABLED and readGains() or GAINS
    pidPitch.kp, pidPitch.ki, pidPitch.kd = gains.kp, gains.ki, gains.kd
    pidRoll.kp,  pidRoll.ki,  pidRoll.kd  = gains.kp, gains.ki, gains.kd

    -- BOUCLE INTERNE : suit la consigne d'inclinaison
    local uPitch = pidPitch:update(pitch, setPitch, dt)
    local uRoll  = pidRoll:update(roll,  setRoll,  dt)

    -- borne la commande a l'echelle redstone
    uPitch = clamp(uPitch, -RS_MAX, RS_MAX)
    uRoll  = clamp(uRoll,  -RS_MAX, RS_MAX)

    -- sens de correction
    local cmdPitch = INVERT_PITCH and -uPitch or uPitch
    local cmdRoll  = INVERT_ROLL  and -uRoll  or uRoll

    -- applique aux thrusters
    driveAxis(cmdPitch, "north", "south")
    driveAxis(cmdRoll,  "east",  "west")

    draw(pitch, roll, cmdPitch, cmdRoll, gains, setPitch, setRoll)
    sleep(DT)
  end
end

-- Lance avec nettoyage garanti (arret propre : toutes les sorties a 0)
local ok, err = pcall(run)
allOff()
term.setCursorPos(1, 1)
if not ok then
  if err == nil then
    print("Arret. Thrusters coupes.")
  else
    print("Erreur : " .. tostring(err))
  end
end
