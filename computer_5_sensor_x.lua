--[[============================================================================
  FUSEE Creat Aeronautics  --  CAPTEUR D'ORIENTATION X ("computer_5")
  ---------------------------------------------------------------------------
  Place a X+3 du principal (computer_8). Role : donner un point de reference
  sur l'axe X du corps de la fusee.

  Boucle : gps.locate() -> broadcast rednet de la position au principal.
  Le principal fait vX = P5 - P8 pour reconstruire l'axe X du corps, puis
  vZ = vX x vY (produit vectoriel) pour l'orientation complete.

  Aucun cablage redstone ici : juste un modem wireless (GPS + rednet).
============================================================================]]--

--============================ CONFIG =======================================--
local ROLE         = "X"           -- NE PAS CHANGER (ce computer = axe X)
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
      term.write(string.format("[X] P5 = %.1f %.1f %.1f  (envois %d)", x, y, z, sent))
    else
      fails = fails + 1
      term.setCursorPos(1, 2)
      term.clearLine()
      term.write("[X] GPS indisponible (echecs " .. fails .. ")")
    end
    sleep(RATE)
  end
end

local ok, err = pcall(run)
if not ok and err then
  print("\nErreur : " .. tostring(err))
end
