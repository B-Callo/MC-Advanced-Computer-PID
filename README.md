# MC-Advanced-Computer-PID

Pilotage complet d'une fusée du mod **Créat Aéronautics** sur **CC:Tweaked**,
réparti sur **4 ordinateurs**. L'orientation est reconstruite **par GPS + produit
vectoriel** (plus de gimbal sensor), puis trois PID en cascade gèrent attitude,
position horizontale et altitude.

## Architecture physique

- **1 vector thruster** sous la fusée (poussée + orientation par redstone).
- **`computer_8`** — ordinateur **principal** : reçoit les positions, calcule
  l'orientation, exécute les 3 PID, pilote les relays, affiche sur l'écran.
- **`computer_6`** — capteur placé à **X+3** du principal (axe X du corps).
- **`computer_9`** — capteur placé à **Y+3** du principal (axe Y = « haut »).
- **`computer_7`** — **terminal** : saisie des cibles + réglage des PID.
- **`redstone_relay_7`** — orientation du thruster (4 directions).
- **`redstone_relay_8`** — puissance (front) + trains d'atterrissage (back).
- **`screen_1`** — écran 1×3 pour la télémétrie.

**Modems** — la communication (`rednet`) passe par le **réseau filaire** qui relie
les ordinateurs entre eux :

- `computer_8` / `computer_6` / `computer_9` : modem **wireless** (obligatoire pour
  `gps.locate()` — le GPS ne fonctionne **pas** en filaire) **+** modem filaire
  pour la comm interne.
- `computer_7` (terminal) : modem **filaire** seul suffit (pas de GPS).

Chaque programme ouvre `rednet` sur **tous** les modems présents. Une
**constellation GPS** (4 hosts) doit être en portée des modems wireless.

## Principe : orientation par produit vectoriel

Chaque ordinateur lit sa position absolue via `gps.locate()`. Le principal
reconstruit deux vecteurs dans le repère du monde :

```
vX = P6 - P8      -> axe X du corps de la fusée
vY = P9 - P8      -> axe Y du corps (le « haut »)
up = normalize(vY)                 -> direction réelle du haut
vZ = vX × vY                       -> axe Z du corps (produit vectoriel)
```

L'**inclinaison** est la part **horizontale** de `up`, projetée sur les axes du
corps → `tiltX`, `tiltZ`. Fusée droite ⇒ `up = (0, 1, 0)` ⇒ tilts nuls.

## Cascade de 3 PID (sur `computer_8`)

```
cible X/Z ─> POSITION (PD) ─> consigne d'inclinaison ─┐
                                                       ▼
                                   ATTITUDE (PID) ─> orientation thruster (relay_7)
cible Y   ─> ALTITUDE (PID) ─────────────────────> puissance thruster (relay_8 front)
```

1. **Attitude** (interne) : suit une consigne d'inclinaison en orientant le
   thruster (4 signaux redstone sur `redstone_relay_7`).
2. **Position** (externe) : écart horizontal cible↔fusée → consigne d'inclinaison
   pour la boucle 1. La fusée s'incline vers la cible, se déplace, puis se
   redresse une fois arrivée.
3. **Altitude** : écart de hauteur → puissance (`redstone_relay_8` front).

## Câblage

**`redstone_relay_7` — orientation du thruster** (signal 0-15 par face) :

| Direction poussée | Face    |
|-------------------|---------|
| vers +X           | `front` |
| vers −X           | `back`  |
| vers +Z           | `right` |
| vers −Z           | `left`  |

**`redstone_relay_8`** :

| Rôle                   | Face    | Valeur              |
|------------------------|---------|---------------------|
| Puissance du thruster  | `front` | 0-15                |
| Trains d'atterrissage  | `back`  | 0 = rentrés, 15 = sortis |

## Fichiers (un par ordinateur)

| Fichier                    | Ordinateur    | Rôle                              |
|----------------------------|---------------|-----------------------------------|
| `computer_8_main.lua`      | `computer_8`  | contrôleur principal (3 PID)      |
| `computer_6_sensor_x.lua`  | `computer_6`  | capteur position, axe X (X+3)     |
| `computer_9_sensor_y.lua`  | `computer_9`  | capteur position, axe Y (Y+3)     |
| `computer_7_terminal.lua`  | `computer_7`  | terminal de commande / tuning     |

**Installer** chaque fichier sur l'ordinateur correspondant, sous le nom
`startup.lua` pour un démarrage automatique. Ex. sur `computer_8` :

```
pastebin get ... startup.lua        # ou wget / edit
```

## Communication (rednet)

Trois protocoles, à garder identiques sur tous les ordis :

| Protocole    | Sens                    | Contenu                                   |
|--------------|-------------------------|-------------------------------------------|
| `rkt_sensor` | capteurs → principal    | `{ role = "X"/"Y", pos = {x,y,z} }`       |
| `rkt_cmd`    | terminal → principal    | cible / arm / trains / gains (partiel)    |
| `rkt_tlm`    | principal → terminal    | télémétrie (position, tilts, puissance…)  |

## Terminal (`computer_7`)

Commandes principales (`help` pour la liste complète) :

```
goto <x> <y> <z>     cible (y = altitude)
x <v> | y <v> | z <v>  change une coordonnée
arm | disarm         arme / désarme le thruster
legs on | legs off   trains d'atterrissage
att <kp> <ki> <kd>   gains attitude
pos <kp> <ki> <kd>   gains position
alt <kp> <ki> <kd>   gains altitude
reseti               remet à zéro les intégrales (anti-windup)
status               télémétrie du principal
resend               renvoie toute la consigne
```

### Persistance (gains + cible)

Les **gains** et la **cible** sont sauvegardés sur disque (`rocket.cfg`) **à chaque
modification**, sur le principal et sur le terminal. Au **reboot**, ils sont
rechargés automatiquement — plus besoin de tout re-régler ni de re-saisir la
cible. Les valeurs de la CONFIG ne servent que de défaut au tout premier
démarrage (fichier absent).

L'état **`armed` n'est jamais persisté** (sécurité) : après un reboot, la fusée
reste **désarmée** tant qu'on n'a pas refait `arm`. Le terminal **n'envoie rien
automatiquement** au démarrage — il ne peut donc pas forcer la cible ni désarmer
la fusée en vol par accident. Utilise `resend` pour repousser cible + gains +
trains (sans toucher à `armed`).

### Réinitialiser les intégrales (`reseti`)

Le terme intégral (`ki`) accumule l'erreur dans le temps. En cas de dérive ou de
saturation (windup) — typiquement après une manœuvre brutale ou une longue
consigne tenue — `reseti` remet à zéro l'intégrale des 5 PID sans toucher aux
gains. Utile aussi juste avant un décollage/atterrissage pour repartir « propre ».

## Réglage / calibration

Régler dans l'ordre, la fusée **armée**, cible à sa position actuelle :

1. **Altitude** (`alt`) — monter `kp` jusqu'à tenir l'altitude, `kd` pour amortir
   le rebond, un peu de `ki` si elle finit sous/au-dessus de la cible. Régler
   aussi `BASE_POWER` dans `computer_8_main.lua` (poussée d'équilibre au hover).
2. **Attitude** (`att`) — monter `kp` jusqu'à ce que la fusée revienne à la
   verticale, `kd` pour amortir l'oscillation. **Si un axe diverge** (la fusée
   part de plus en plus fort), basculer `ATT_INVERT_X` / `ATT_INVERT_Z` dans la
   CONFIG (le sens de correction est faux).
3. **Position** (`pos`) — activer une cible décalée : la fusée doit **revenir**
   vers elle. Si elle **s'éloigne**, basculer `POS_INVERT_X` / `POS_INVERT_Z`.
   Trop de dépassement / oscillation autour de la cible ⇒ baisser `kp` ou monter
   `kd`. `MAX_TILT` limite l'inclinaison max demandée (agressivité).

**Sécurité** : `disarm` (ou Ctrl+T sur le principal) coupe la poussée et
l'orientation. Si les capteurs de bras se taisent (`RX_TIMEOUT`), le principal
maintient le hover mais arrête le vectoring et le signale (`capteurs?`).

## Notes

- La résolution d'orientation dépend du bras de levier (3 blocs) et de la
  précision du GPS : viser un hover posé plutôt que des acrobaties.
- `DT`, `FILTER`, `GPS_TIMEOUT` et les timeouts sont en haut de chaque fichier.
