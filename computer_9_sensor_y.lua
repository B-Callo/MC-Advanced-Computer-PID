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

local modem = peripheral.find("modem", function(_, m) return m.isWireless() end)
if not modem then error("Aucun modem wireless (necessaire GPS + rednet).", 0) end
rednet.open(peripheral.getName(modem))

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

local ok, err = pcall(run)
if not ok and err then
  print("\nErreur : " .. tostring(err))
end
