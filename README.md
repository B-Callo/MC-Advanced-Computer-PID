# MC-Advanced-Computer-PID

Contrôleur **PID d'attitude** pour une fusée du mod **Créat Aéronautics**, tournant
sur un **Advanced Computer** de CC:Tweaked (Minecraft).

Objectif : garder la fusée **droite en stationnaire** (hover) par *thrust vectoring* —
le PID oriente le vector thruster pour annuler l'inclinaison mesurée par le gimbal sensor.
Une **boucle de position** optionnelle la maintient en plus **au-dessus d'une ancre**.

Architecture en **cascade** :

```
navigation table ──> BOUCLE EXTERNE (position/PD) ──> consigne d'inclinaison
                                                           │
gimbal sensor ─────> BOUCLE INTERNE (attitude/PID) <───────┘ ──> vector thruster
```

## Fichiers

- `startup.lua` — programme complet (config + PID + boucle). Nommé `startup` pour
  démarrer automatiquement au boot du computer.

## Câblage

Quatre redstone relays :

| Variable     | Rôle                                                    |
|--------------|---------------------------------------------------------|
| `RELAY_IN`   | entrées gimbal sensor (4 directions)                    |
| `RELAY_OUT`  | sorties vector thruster (4 directions)                  |
| `RELAY_TUNE` | réglage live des gains (kp / ki / kd)                   |
| `RELAY_NAV`  | navigation table : offset vers l'ancre (4 dir + dist)   |

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

1. **`RELAY_IN` / `RELAY_OUT` / `RELAY_TUNE` / `RELAY_NAV`** — noms des relays
   (voir `peripheral.getNames()` en jeu). Mets `"computer"` pour utiliser
   directement les faces de l'ordinateur.
2. **`INPUTS` / `OUTPUTS` / `NAV`** — la `side` redstone de chaque signal
   (`top`, `bottom`, `left`, `right`, `front`, `back`).
3. **`GAINS`** (attitude) et **`NAV_GAINS`** (position).
4. **`INVERT_PITCH` / `INVERT_ROLL`** et **`NAV_INVERT_NS` / `NAV_INVERT_EW`** —
   sens de correction (voir tuning / calibration).

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

## Maintien au-dessus de l'ancre (navigation table)

Avec `NAV_ENABLED = true`, une **boucle externe** reconstruit l'écart à l'ancre
depuis `RELAY_NAV`, puis un PD par axe le transforme en **consigne d'inclinaison** :
la fusée s'incline vers l'ancre pour y revenir, puis se redresse une fois au-dessus.
`NAV_ENABLED = false` → simple hover vertical.

L'écart est reconstruit en combinant deux sources (elles ne portent pas la même
info) :

- **Direction** — les 4 signaux directionnels de la nav table indiquent *où* est
  l'ancre (ex : `north` actif = ancre au nord), **pas** la distance. Normalisés,
  ils donnent la direction sur chaque axe : `dir = (north − south) / 15`.
- **Éloignement** — le **modulating link** (signal `dist`, dessous) donne la
  distance : `15` = pile sur l'ancre, `0` = très loin / hors de portée. On en
  déduit `farness = 15 − dist`.

Écart utilisé par le PD : `e = dir × farness` (direction × éloignement).

Réglages (CONFIG) :

- **`NAV_GAINS`** `{ kp, kd }` — force du retour (`kp`) et amortissement (`kd`).
- **`NAV_MAX_TILT`** — inclinaison maximale demandée (limite l'agressivité).
- **`NAV_DEADBAND`** — zone morte : n'incline pas pour un offset minuscule.
- **`NAV_INVERT_NS` / `NAV_INVERT_EW`** — sens du retour (voir calibration).

**Calibration** (à faire après avoir réglé la boucle d'attitude) :

1. Règle d'abord l'attitude seule (`NAV_ENABLED = false`) : la fusée doit tenir
   la verticale proprement.
2. Active la nav. Décale la fusée à la main : elle doit **revenir vers l'ancre**.
   Si elle **s'éloigne** sur un axe, inverse `NAV_INVERT_NS` / `NAV_INVERT_EW`.
3. Ça dépasse / oscille autour de l'ancre ? Baisse `NAV_GAINS.kp` ou monte `kd`.
   Trop mou / trop lent ? Monte `kp`.

L'écran affiche `csg` (consigne d'inclinaison calculée) et la ligne `NAV ancre`
avec la distance et les 4 signaux bruts — utile pour calibrer.

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
