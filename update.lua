--[[============================================================================
  FUSEE Creat Aeronautics  --  MISE A JOUR AUTOMATIQUE
  ---------------------------------------------------------------------------
  Chaque machine memorise son ROLE dans le fichier role.cfg. A definir UNE FOIS
  par machine (ensuite 'update' seul sait quoi telecharger) :

    update main        ordinateur principal      -> computer_8_main.lua
    update sensorx     capteur axe X (X+3)        -> computer_6_sensor_x.lua
    update sensory     capteur axe Y (Y+3)        -> computer_9_sensor_y.lua
    update terminal    terminal de commande       -> computer_7_terminal.lua

  Ensuite :
    update             met a jour cette machine selon son role memorise
    update all         diffuse l'ordre a TOUTE la flotte (chaque machine se met
                       a jour selon SON role), puis met a jour cette machine

  Le fichier telecharge ecrase toujours l'ancien startup.lua.
  ==> On ne se base PLUS sur l'ID de l'ordinateur (peu fiable) mais sur le role.
============================================================================]]--

local REPO = "https://raw.githubusercontent.com/B-Callo/MC-Advanced-Computer-PID/main/"

-- role -> fichier source
local ROLES = {
  main     = "computer_8_main.lua",
  sensorx  = "computer_6_sensor_x.lua",
  sensory  = "computer_9_sensor_y.lua",
  terminal = "computer_7_terminal.lua",
}

local ROLE_FILE    = "role.cfg"
local PROTO_UPDATE = "rkt_update"

--------------------------------------------------------------------------------
local function openModems()
  for _, name in ipairs(peripheral.getNames()) do
    if peripheral.getType(name) == "modem" then rednet.open(name) end
  end
end

local function readRole()
  if not fs.exists(ROLE_FILE) then return nil end
  local f = fs.open(ROLE_FILE, "r"); local r = f.readAll(); f.close()
  r = (r or ""):gsub("%s+", ""):lower()
  return ROLES[r] and r or nil
end

local function saveRole(r)
  local f = fs.open(ROLE_FILE, "w"); f.write(r); f.close()
end

-- Telecharge le fichier correspondant au role dans startup.lua (ecrase l'ancien).
local function download(role)
  local url = REPO .. ROLES[role]
  print("Role '" .. role .. "' -> " .. ROLES[role])
  local resp = http.get(url)
  if not resp then
    print("Echec HTTP. Verifie l'acces a raw.githubusercontent.com."); return false
  end
  local data = resp.readAll(); resp.close()
  if not data or #data == 0 then print("Reponse vide. Abandon."); return false end
  if fs.exists("startup.lua") then fs.delete("startup.lua") end
  local f = fs.open("startup.lua", "w"); f.write(data); f.close()
  print(string.format("startup.lua mis a jour (%d octets).", #data))
  return true
end

--------------------------------------------------------------------------------
local args = { ... }
local wantAll, roleArg = false, nil
for _, a in ipairs(args) do
  a = a:lower()
  if a == "all" then wantAll = true
  elseif ROLES[a] then roleArg = a
  else print("Argument inconnu : " .. a) end
end

-- Diffuse aux autres machines EN PREMIER (meme si cette machine n'a pas de role
-- defini : la flotte se met quand meme a jour).
if wantAll then
  openModems()
  rednet.broadcast("update", PROTO_UPDATE)
  print("Ordre d'update diffuse a toute la flotte.")
  sleep(0.5)
end

-- Determine le role de cette machine : argument explicite, sinon celui memorise.
local role = roleArg or readRole()
if not role then
  print("Role inconnu pour cette machine.")
  print("Definis-le une fois :  update <role>")
  print("Roles : main | sensorx | sensory | terminal")
  return
end
saveRole(role)   -- memorise pour les prochaines fois

if download(role) then
  print("Reboot dans 2 s...  (Ctrl+T pour annuler)")
  sleep(2)
  os.reboot()
end
