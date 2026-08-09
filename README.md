# MC-Advanced-Computer-PID

Contrôleur **PID d'attitude** pour une fusée du mod **Créat Aéronautics**, tournant
sur un **Advanced Computer** de CC:Tweaked (Minecraft).

Objectif : garder la fusée **droite en stationnaire** (hover) par *thrust vectoring* —
le PID oriente le vector thruster pour annuler l'inclinaison mesurée par le gimbal sensor.

## Fichiers

- `startup.lua` — programme complet (config + PID + boucle). Nommé `startup` pour
  démarrer automatiquement au boot du computer.

## Câblage

Trois redstone relays :

| Variable     | Rôle                                    |
|--------------|-----------------------------------------|
| `RELAY_IN`   | entrées gimbal sensor (4 directions)    |
| `RELAY_OUT`  | sorties vector thruster (4 directions)  |
| `RELAY_TUNE` | réglage live des gains (kp / ki / kd)   |

**Entrées — gimbal sensor (4 signaux redstone analogiques 0-15) :**
une valeur par direction d'inclinaison.

| Direction | Signification            |
|-----------|--------------------------|
| `north`   | inclinaison vers le nord |
| `south`   | inclinaison vers le sud  |
| `east`    | inclinaison vers l'est   |
| `west`    | inclinaison vers l'ouest |

**Sorties — vector thruster (4 signaux redstone) :** une par direction de poussée.
Câblées ici sur un **redstone relay** (peripheral).

> On ne contrôle **que l'orientation** du thruster, pas la puissance de poussée.

Axes de contrôle :
- **Tangage (pitch)** = `north − south`
- **Roulis (roll)** = `east − west`

## Configuration

Tout est en haut de `startup.lua`, section `CONFIG` :

1. **`RELAY`** — nom du redstone relay (voir `peripheral.getNames()` en jeu).
   Mets `"computer"` pour utiliser directement les faces de l'ordinateur.
2. **`INPUTS` / `OUTPUTS`** — la `side` redstone de chaque signal
   (`top`, `bottom`, `left`, `right`, `front`, `back`).
3. **`GAINS`** — `kp`, `ki`, `kd`.
4. **`INVERT_PITCH` / `INVERT_ROLL`** — sens de correction (voir tuning).

## Réglage live des gains (relay de tune)

Avec `TUNE_ENABLED = true`, les gains sont lus **en continu** sur `RELAY_TUNE`
(3 faces, un signal 0-15 par gain). Tu peux donc régler le PID à chaud avec des
leviers/comparateurs sans rebooter. Le mapping :

```
gain = signal_redstone / 15 * TUNE_MAX[gain]
```

`TUNE_MAX` (dans la CONFIG) fixe la valeur atteinte à plein signal (15) :
`kp` → 0..4.0, `ki` → 0..1.0, `kd` → 0..2.0 par défaut. Ajuste ces plafonds si
tu as besoin de plus de course.

Mets `TUNE_ENABLED = false` pour figer les gains sur les valeurs de `GAINS`.

## Tuning (réglage des gains)

1. Mets `ki = 0` et `kd = 0` (faces ki/kd du relay à 0). Augmente `kp` jusqu'à ce
   que la fusée réagisse et revienne vers la verticale.
2. **Si un axe diverge** (part de plus en plus fort au lieu de se corriger) :
   passe `INVERT_PITCH` ou `INVERT_ROLL` à `true` (ou inverse les câbles de
   cette paire de sorties). Le signe de correction est faux.
3. Ça oscille autour de la verticale ? Monte `kd` pour amortir.
4. Il reste un léger biais permanent ? Ajoute un peu de `ki`.

Autres réglages : `DT` (période de boucle), `FILTER` (lissage capteur),
`DEADBAND` (zone morte anti-jitter).

## Installation sur le computer

Sur l'Advanced Computer en jeu :

```
wget https://raw.githubusercontent.com/B-Callo/MC-Advanced-Computer-PID/main/startup.lua startup.lua
```

Puis `reboot` (ou lance `startup`). **Ctrl+T** arrête le programme et coupe
proprement toutes les sorties.
