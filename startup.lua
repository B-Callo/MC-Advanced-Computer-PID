--[[============================================================================
  PID d'attitude pour fusee Creat Aeronautics  --  CC:Tweaked Advanced Computer
  ---------------------------------------------------------------------------
  Objectif : garder la fusee droite en stationnaire (hover) par thrust vectoring.

  Principe :
    - Le "gimbal sensor" renvoie 4 signaux redstone analogiques (0-15), un par
      direction d'inclinaison : nord, sud, est, ouest.
    - On calcule l'inclinaison nette sur 2 axes :
          tangage (pitch) = nord - sud
          roulis  (roll)  = est  - ouest
    - Un PID par axe ramene cette inclinaison a 0.
    - La sortie du PID oriente le "vector thruster" via 4 signaux redstone
      (cables sur un redstone relay), un par direction.

  ==> Tout se configure dans la section CONFIG juste en dessous.
============================================================================]]--

--============================ CONFIG (a remplir) ============================--

-- Nom du peripheral redstone relay tel qu'affiche par CC.
-- Mets "computer" pour utiliser directement les 6 faces de l'ordinateur.
local RELAY = "redstone_relay_0"

-- ENTREES : gimbal sensor. Une face par direction d'inclinaison.
-- device = RELAY (le relay) ou "computer" (face de l'ordi) ; side = face redstone.
local INPUTS = {
  north = { device = RELAY, side = "top"    },
  south = { device = RELAY, side = "bottom" },
  east  = { device = RELAY, side = "left"   },
  west  = { device = RELAY, side = "right"  },
}

-- SORTIES : orientation du vector thruster. Une face par direction de poussee.
local OUTPUTS = {
  north = { device = RELAY, side = "front" },
  south = { device = RELAY, side = "back"  },
  east  = { device = RELAY, side = "left"  },
  west  = { device = RELAY, side = "right" },
}

-- Gains PID (identiques sur les 2 axes par defaut ; separe-les si besoin).
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
-- Controleur PID (setpoint fixe = 0 : on veut zero inclinaison)
--------------------------------------------------------------------------------
local function newPID(g)
  return {
    kp = g.kp, ki = g.ki, kd = g.kd,
    integral = 0,
    prevError = 0,
    update = function(self, measurement, dt)
      -- setpoint = 0  =>  erreur = -mesure
      local err = -measurement

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
local filt = { north = 0, south = 0, east = 0, west = 0 }

local function readFiltered(key)
  local raw = readAnalog(INPUTS[key])
  filt[key] = FILTER * raw + (1 - FILTER) * filt[key]
  return filt[key]
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
-- Affichage
--------------------------------------------------------------------------------
local function draw(pitch, roll, uPitch, uRoll)
  term.clear()
  term.setCursorPos(1, 1)
  print("=== PID Attitude Fusee (Ctrl+T pour arreter) ===")
  print("")
  print(string.format(" TANGAGE (N-S) : inclin=%+.2f  cmd=%+.2f", pitch, uPitch))
  print(string.format(" ROULIS  (E-O) : inclin=%+.2f  cmd=%+.2f", roll,  uRoll))
  print("")
  print(string.format(" Gains  Kp=%.2f  Ki=%.2f  Kd=%.2f", GAINS.kp, GAINS.ki, GAINS.kd))
end

--------------------------------------------------------------------------------
-- Boucle de controle
--------------------------------------------------------------------------------
local function run()
  local pidPitch = newPID(GAINS)
  local pidRoll  = newPID(GAINS)

  allOff()
  local last = os.clock()

  while true do
    -- dt reel
    local now = os.clock()
    local dt = now - last
    if dt <= 0 then dt = DT end
    last = now

    -- inclinaison nette par axe
    local pitch = readFiltered("north") - readFiltered("south")
    local roll  = readFiltered("east")  - readFiltered("west")

    -- deadband anti-jitter
    if math.abs(pitch) <= DEADBAND then pitch = 0 end
    if math.abs(roll)  <= DEADBAND then roll  = 0 end

    -- PID
    local uPitch = pidPitch:update(pitch, dt)
    local uRoll  = pidRoll:update(roll,  dt)

    -- borne la commande a l'echelle redstone
    uPitch = clamp(uPitch, -RS_MAX, RS_MAX)
    uRoll  = clamp(uRoll,  -RS_MAX, RS_MAX)

    -- sens de correction
    local cmdPitch = INVERT_PITCH and -uPitch or uPitch
    local cmdRoll  = INVERT_ROLL  and -uRoll  or uRoll

    -- applique aux thrusters
    driveAxis(cmdPitch, "north", "south")
    driveAxis(cmdRoll,  "east",  "west")

    draw(pitch, roll, cmdPitch, cmdRoll)
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
