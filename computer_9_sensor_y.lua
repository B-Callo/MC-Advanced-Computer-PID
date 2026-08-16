--[[============================================================================
  FUSEE Creat Aeronautics  --  CAPTEUR D'ORIENTATION Y ("computer_9")
  ---------------------------------------------------------------------------
  Place a Y+3 du principal (computer_8). Role : donner un point de reference
  sur l'axe Y du corps (le "haut" de la fusee).

  Boucle : gps.locate() -> broadcast rednet de la position au principal.
  Le principal fait vY = P9 - P8 : c'est l'axe vertical de la fusee dans le
  monde. up = normalize(vY) donne l'inclinaison ; l'ecart de up par rapport a
  la verticale (0,1,0) = a quel point la fusee penche.

  Aucun cablage redstone ici : juste un modem wireless (GPS + rednet).
============================================================================]]--

--============================ CONFIG =======================================--
local ROLE         = "Y"           -- NE PAS CHANGER (ce computer = axe Y / haut)
local RATE         = 0.1           -- periode d'envoi (s)
local GPS_TIMEOUT  = 0.4           -- timeout gps.locate() (s)
local PROTO_SENSOR = "rkt_sensor"  -- doit matcher le principal
--===========================================================================--

-- Ouvre rednet sur TOUS les modems (filaire pour la comm interne). Le modem
-- WIRELESS reste indispensable pour gps.locate() (le GPS ne marche pas en filaire).
local opened = 0
for _, name in ipairs(peripheral.getNames()) do
  if peripheral.getType(name) == "modem" then
    rednet.open(name)
    opened = opened + 1
  end
end
if opened == 0 then error("Aucun modem : rednet impossible.", 0) end

term.clear()
local sent, fails = 0, 0

local function run()
  while true do
    local x, y, z = gps.locate(GPS_TIMEOUT)
    if x then
      rednet.broadcast({ role = ROLE, pos = { x = x, y = y, z = z } }, PROTO_SENSOR)
      sent = sent + 1
      term.setCursorPos(1, 1)
      term.clearLine()
      term.write(string.format("[Y] P9 = %.1f %.1f %.1f  (envois %d)", x, y, z, sent))
    else
      fails = fails + 1
      term.setCursorPos(1, 2)
      term.clearLine()
      term.write("[Y] GPS indisponible (echecs " .. fails .. ")")
    end
    sleep(RATE)
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

local ok, err = pcall(parallel.waitForAny, run, updateListener)
if doUpdate then
  print("\nMise a jour recue...")
  shell.run("update")
elseif not ok and err then
  print("\nErreur : " .. tostring(err))
end
