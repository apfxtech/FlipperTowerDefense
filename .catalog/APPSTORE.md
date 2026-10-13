# Tower Defense

A full tower defense on the 128x64 one-bit screen. Five hand-built maps, eight towers, eleven kinds of creep and waves that never stop coming: every round is longer, heavier and meaner than the last, and the only question is how far you get before the last life is gone.

Tower Defense is a Flipper Zero port of MicroTD, written for the Arduboy by Miloslav "drummyfish" Ciz and released into the public domain. The port by ApertureFox Technology is native Flipper code: a one-bit-per-pixel framebuffer drawn straight onto the Flipper canvas at 60 frames per second, speaker tones, and a best round per map kept on the SD card.

## Maps

Five maps, each with its own creep path, starting purse, starting lives and spawn table. The first runs a single path; the rest run two at once, from opposite corners or forking out of one entrance, and creeps walk both of them at the same time.

- **Map 1** - one path, 20 lives, $10 to start.
- **Map 2** - two paths that merge into a shared exit, 25 lives, $10.
- **Map 3** - two separate paths from opposite corners, 20 lives, $10.
- **Map 4** - a single entrance whose path forks left and right at the far end, 10 lives, $8. The tightest map in the game.
- **Map 5** - two paths snaking across the board in opposite directions, the heaviest spawn table, 30 lives, $22.

Each map keeps its own record, shown under the map name in the menu and stored on the SD card. Waves are deterministic, so the same map always plays out the same way and a better round is a better plan, not a luckier roll.

## Towers

Eight types. Each has two upgrades, both costing two thirds of the build price, and selling one back returns half of what it cost. Towers go up between waves, never during one.

- **Guard** ($8) - cheap, fast, short range. Upgrades: range, attack speed.
- **Cannon** ($8) - splashes half damage onto everything near its target, but cannot touch bats. Upgrades: range, damage.
- **Ice** ($17) - deals no damage at all. Freezes a creep in place, then immediately hunts for the next unfrozen one. Upgrades: attack speed, range.
- **Electro** ($30) - heavy single-target damage at good range. Upgrades: damage, shock (a one in three chance to freeze on every hit).
- **Sniper** ($45) - reaches nearly half the screen. Upgrades: attack speed, range.
- **Magic** ($60) - modest damage, but the aura upgrade speeds up every non-magic tower in the surrounding 3x3. Upgrades: damage, aura.
- **Water** ($100) - two tiles by two, heavy damage, and its wave knocks creeps back down the path. Upgrades: range, attack speed.
- **Fire** ($100) - two tiles by two, the hardest hitter in the game, with splash. Upgrades: range, damage.

While an item is selected in the action bar, the tower and the circle of its range are previewed on the tile under the cursor, and a range or aura upgrade shows what it would add before you pay for it.

## Creeps

Eleven types, from a 7 health spider to 63 health dinosaurs and demons. Later rounds cycle through the higher tiers and hand extra lives to the leader of each group: a creep like that is killed, respawns at the entrance with full health, and has to be killed again, up to three times over.

Five of them break the rules:

- **Wolf** - shrugs off freezing in half the time.
- **Bat** - immune to cannons.
- **Big spider** - splits into two small spiders when it dies.
- **Ghost** - immune to every physical tower: guard, cannon and sniper.
- **Demon** - only water and fire can touch it at all.

Killing a creep pays 1 to 3 dollars, surviving a wave pays 5 plus a quarter of the round number, and a creep that reaches the exit costs you as many lives as it would have paid.

## Controls

On the map:

- **Arrows** - move the cursor over the grid
- **OK between waves** - open the action bar for the tile under the cursor
- **OK held during a wave** - show a health bar over every creep
- **OK + Back during a wave** - fast forward, five times normal speed
- **Back held** - hide the info bar to see the tiles behind it
- **Back long** - leave the map and return to the main menu

In the action bar along the bottom:

- **Left / Right** - pick an action: eight towers, two upgrades, sell, next wave, quit
- **OK** - confirm. Selling a tower and quitting ask first
- **Back** - close the bar

In the main menu:

- **Left / Right** - choose a map, or the sound switch
- **OK** - start the map, or toggle sound
- **Back long** - exit the game

Sound starts switched off when the Flipper is in stealth mode, and can be turned back on from the menu at any time.
