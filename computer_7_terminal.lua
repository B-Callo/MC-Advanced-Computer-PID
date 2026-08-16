--[[============================================================================
  FUSEE Creat Aeronautics  --  TERMINAL DE COMMANDE ("computer_7")
  ---------------------------------------------------------------------------
  Interface pour :
    - entrer la cible (X, altitude Y, Z)
    - armer / desarmer, sortir / rentrer les trains
    - regler les gains des 3 PID en direct (attitude / position / altitude)
  Envoie tout au principal (computer_8) par rednet, et affiche la telemetrie
  qu'il renvoie.

  Tape "help" pour la liste des commandes. Modem wireless requis.
============================================================================]]--

--============================ CONFIG =======================================--
local PROTO_CMD = "rkt_cmd"   -- -> principal
local PROTO_TLM = "rkt_tlm"   -- <- principal (telemetrie)
--===========================================================================--

-- Ouvre rednet sur TOUS les modems presents (ici : le modem filaire local
-- qui relie le terminal au reseau interne de la fusee).
local opened = 0
for _, name in ipairs(peripheral.getNames()) do
  if peripheral.getType(name) == "modem" then
    rednet.open(name)
    opened = opened + 1
  end
end
if opened == 0 then error("Aucun modem : rednet impossible.", 0) end

-- Etat local (miroir de la consigne envoyee au principal).
local cmd = {
  target = { x = 0, y = 100, z = 0 },
  armed  = false,
  legs   = false,
  gains  = {
    att = { kp = 6.0, ki = 0.0, kd = 3.0 },
    pos = { kp = 0.35, ki = 0.0, kd = 0.80 },
    alt = { kp = 2.0, ki = 0.15, kd = 1.5 },
  },
}
local lastTlm = nil

local function send(partial) rednet.broadcast(partial, PROTO_CMD) end

-- Persistance sur disque (gains + cible + trains) : survit au reboot du terminal.
-- On NE persiste PAS 'armed' (securite : le terminal ne doit jamais rearmer seul).
local CFG_FILE = "rocket.cfg"
local function saveCfg()
  local f = fs.open(CFG_FILE, "w")
  if f then
    f.write(textutils.serialize({ gains = cmd.gains, target = cmd.target, legs = cmd.legs }))
    f.close()
  end
end
local function loadCfg()
  if not fs.exists(CFG_FILE) then return end
  local f = fs.open(CFG_FILE, "r")
  if not f then return end
  local data = textutils.unserialize(f.readAll() or "")
  f.close()
  if type(data) ~= "table" then return end
  if type(data.gains) == "table" then
    if type(data.gains.att) == "table" then cmd.gains.att = data.gains.att end
    if type(data.gains.pos) == "table" then cmd.gains.pos = data.gains.pos end
    if type(data.gains.alt) == "table" then cmd.gains.alt = data.gains.alt end
  end
  if type(data.target) == "table" then cmd.target = data.target end
  if type(data.legs) == "boolean" then cmd.legs = data.legs end
end
loadCfg()

--------------------------------------------------------------------------------
-- Aide
--------------------------------------------------------------------------------
local function help()
  print("Commandes :")
  print("  goto <x> <y> <z>   cible (y = altitude)")
  print("  x <v> | y <v> | z <v>   change une coordonnee cible")
  print("  arm | disarm       arme / desarme le thruster")
  print("  legs on | legs off trains d'atterrissage")
  print("  att <kp> <ki> <kd> gains attitude")
  print("  pos <kp> <ki> <kd> gains position")
  print("  alt <kp> <ki> <kd> gains altitude")
  print("  gains              affiche les gains courants")
  print("  reseti             remet a zero les integrales (anti-windup)")
  print("  status             telemetrie recue du principal")
  print("  resend             renvoie cible + gains + trains")
  print("  update             met a jour TOUTE la flotte (rednet) + reboot")
  print("  help | exit")
end

--------------------------------------------------------------------------------
-- Affichage telemetrie
--------------------------------------------------------------------------------
local function fmtVec(p)
  if not p then return "?" end
  return string.format("%.1f, %.1f, %.1f", p.x, p.y, p.z)
end

local function showStatus()
  local t = lastTlm
  if not t then print("Pas encore de telemetrie recue."); return end
  print("--- Telemetrie principal ---")
  print("  Etat    : " .. (t.armed and "ARMEE" or "securite")
        .. (t.ok == false and "  (capteurs perimes!)" or ""))
  print("  Position: " .. fmtVec(t.P8))
  print("  Cible   : " .. fmtVec(t.target))
  print(string.format("  Tilt    : X=%+.3f Z=%+.3f", t.tiltX or 0, t.tiltZ or 0))
  print(string.format("  Consigne: X=%+.3f Z=%+.3f", t.setTiltX or 0, t.setTiltZ or 0))
  print(string.format("  Bras    : |X|=%.2f |Y|=%.2f  (attendu ~3)",
        t.lenX or 0, t.lenY or 0))
  print(string.format("  Age GPS : X=%.1fs Y=%.1fs", t.ageX or 9, t.ageY or 9))
  print(string.format("  Puissance %d/15   Trains: %s",
        t.power or 0, t.legs and "sortis" or "rentres"))
end

local function showGains()
  local g = cmd.gains
  print(string.format("att kp=%.2f ki=%.2f kd=%.2f", g.att.kp, g.att.ki, g.att.kd))
  print(string.format("pos kp=%.2f ki=%.2f kd=%.2f", g.pos.kp, g.pos.ki, g.pos.kd))
  print(string.format("alt kp=%.2f ki=%.2f kd=%.2f", g.alt.kp, g.alt.ki, g.alt.kd))
end

--------------------------------------------------------------------------------
-- Parsing des commandes
--------------------------------------------------------------------------------
local function num(s) return tonumber(s) end

local function setGain(key, a, b, c)
  local kp, ki, kd = num(a), num(b), num(c)
  if not (kp and ki and kd) then print("Usage: " .. key .. " <kp> <ki> <kd>"); return end
  cmd.gains[key] = { kp = kp, ki = ki, kd = kd }
  saveCfg()   -- persiste cote terminal
  send({ gains = { [key] = cmd.gains[key] } })
  print(key .. " -> kp=" .. kp .. " ki=" .. ki .. " kd=" .. kd)
end

local function handle(input)
  local w = {}
  for tok in string.gmatch(input, "%S+") do w[#w + 1] = tok end
  local c = (w[1] or ""):lower()

  if c == "help" or c == "?" then help()
  elseif c == "goto" then
    local x, y, z = num(w[2]), num(w[3]), num(w[4])
    if not (x and y and z) then print("Usage: goto <x> <y> <z>"); return end
    cmd.target = { x = x, y = y, z = z }
    saveCfg()
    send({ target = cmd.target })
    print("Cible -> " .. fmtVec(cmd.target))
  elseif c == "x" or c == "y" or c == "z" then
    local v = num(w[2]); if not v then print("Usage: " .. c .. " <valeur>"); return end
    cmd.target[c] = v
    saveCfg()
    send({ target = cmd.target })
    print("Cible -> " .. fmtVec(cmd.target))
  elseif c == "arm" then
    cmd.armed = true;  send({ armed = true });  print(">> ARMEE")
  elseif c == "disarm" then
    cmd.armed = false; send({ armed = false }); print(">> securite")
  elseif c == "legs" then
    local on = (w[2] or ""):lower() == "on"
    cmd.legs = on; saveCfg(); send({ legs = on }); print("Trains: " .. (on and "sortis" or "rentres"))
  elseif c == "att" or c == "pos" or c == "alt" then
    setGain(c, w[2], w[3], w[4])
  elseif c == "gains" then showGains()
  elseif c == "reseti" then
    send({ resetI = true }); print("Integrales remises a zero.")
  elseif c == "status" then showStatus()
  elseif c == "update" then
    print("Mise a jour de toute la flotte...")
    shell.run("update", "all")   -- diffuse + met a jour ce terminal + reboot
  elseif c == "resend" then
    -- volontairement SANS 'armed' (evite de desarmer par accident)
    send({ target = cmd.target, gains = cmd.gains, legs = cmd.legs })
    print("Cible + gains + trains renvoyes (arm inchange).")
  elseif c == "exit" then error("__exit__", 0)
  elseif c == "" then -- rien
  else print("Inconnu : " .. c .. "  (help)")
  end
end

--------------------------------------------------------------------------------
-- Deux taches en parallele : saisie clavier + reception telemetrie
--------------------------------------------------------------------------------
local function inputLoop()
  term.clear(); term.setCursorPos(1, 1)
  print("=== Terminal Fusee (computer_7) ===")
  help()
  -- Pas d'envoi automatique au demarrage : le principal garde sa propre cible
  -- (persistee) et son etat 'armed'. Un 'send(cmd)' ici forcerait la cible a la
  -- valeur locale et DESARMERAIT la fusee en vol. Utilise 'resend' si besoin.
  while true do
    write("\n> ")
    local line = read()
    local ok, err = pcall(handle, line)
    if not ok then
      if err == "__exit__" then return end
      print("Erreur: " .. tostring(err))
    end
  end
end

local function telemetryLoop()
  while true do
    local _, msg, proto = rednet.receive(PROTO_TLM)
    if type(msg) == "table" then lastTlm = msg end
  end
end

-- Ecoute un ordre de mise a jour (rednet) et se re-telecharge.
local doUpdate = false
local function updateListener()
  while true do
    local _, msg = rednet.receive("rkt_update")
    if msg == "update" then doUpdate = true; return end
  end
end

parallel.waitForAny(inputLoop, telemetryLoop, updateListener)
if doUpdate then
  print("\nMise a jour recue...")
  shell.run("update")
end
term.setCursorPos(1, term.getSize() and select(2, term.getSize()) or 1)
print("\nTerminal ferme.")
