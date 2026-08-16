--[[============================================================================
  FUSEE Creat Aeronautics  --  MISE A JOUR AUTOMATIQUE
  ---------------------------------------------------------------------------
  Lance ce script pour re-telecharger le bon programme et rebooter.

    update          met a jour SEULEMENT cette machine (detecte via l'ID)
    update all      diffuse l'ordre a TOUTES les machines du reseau rednet,
                    puis met a jour cette machine

  Le fichier telecharge ecrase toujours l'ancien startup.lua.
  Mapping ID d'ordinateur -> fichier source (l'ID = le "N" de computer_N).
============================================================================]]--

local REPO = "https://raw.githubusercontent.com/B-Callo/MC-Advanced-Computer-PID/main/"

local FILES = {
  [8] = "computer_8_main.lua",       -- principal
  [6] = "computer_6_sensor_x.lua",   -- capteur X
  [9] = "computer_9_sensor_y.lua",   -- capteur Y
  [7] = "computer_7_terminal.lua",   -- terminal
}

local PROTO_UPDATE = "rkt_update"

--------------------------------------------------------------------------------
local function openModems()
  for _, name in ipairs(peripheral.getNames()) do
    if peripheral.getType(name) == "modem" then rednet.open(name) end
  end
end

-- Telecharge le fichier de CETTE machine dans startup.lua (ecrase l'ancien).
local function selfUpdate()
  local id = os.getComputerID()
  local file = FILES[id]
  if not file then
    print("ID d'ordinateur " .. id .. " inconnu (voir table FILES). Abandon.")
    return false
  end
  local url = REPO .. file
  print("Telechargement : " .. file)
  local resp = http.get(url)
  if not resp then
    print("Echec HTTP. Verifie l'acces a raw.githubusercontent.com.")
    return false
  end
  local data = resp.readAll()
  resp.close()
  if not data or #data == 0 then print("Reponse vide. Abandon."); return false end

  if fs.exists("startup.lua") then fs.delete("startup.lua") end
  local f = fs.open("startup.lua", "w")
  f.write(data)
  f.close()
  print(string.format("startup.lua mis a jour (%d octets).", #data))
  return true
end

--------------------------------------------------------------------------------
local args = { ... }

if args[1] == "all" then
  openModems()
  rednet.broadcast("update", PROTO_UPDATE)
  print("Ordre d'update diffuse a toute la flotte.")
  sleep(0.5)   -- laisse le temps aux autres de recevoir
end

if selfUpdate() then
  print("Reboot dans 2 s...  (Ctrl+T pour annuler)")
  sleep(2)
  os.reboot()
end
