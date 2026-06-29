# ניווט אוטונומי — Kobuki (The Construct)

פרויקט לקורס **Autonomous Vehicles**: רובוט Kobuki בונה מפה, מתמקם, ומנווט לנקודת יעד בודדת תוך הימנעות ממכשולים.

**סביבה:** [The Construct ROSDS](https://app.theconstructsim.com/) — ROS 1 Noetic, סימולציית Kobuki + RViz.  
**מחסנית:** gmapping (SLAM) · AMCL · move_base (navfn + DWA).

> שם החבילה `summit_xl_autonomous_nav` נשמר מסיבות היסטוריות; הקוד מותאם ל-**Kobuki**, לא ל-Summit XL.

## זרימת הדמו (definition of done)

1. הפעלת סימולציית הקורס ב-The Construct.
2. **מיפוי אוטומטי** — הרובוט סורק את החדר (מרכזי שטח פתוח + סיבוב 360°), עד ~200 שניות או כיסוי מלא.
3. (אופציונלי) **מיפוי ידני** — שליטה במקלדת לתיקון/השלמת אזורים (למשל מטבח).
4. **שמירת מפה** — `maps/summit_world.pgm` + `maps/summit_world.yaml`.
5. **ניווט** — טעינת המפה (map_server + AMCL + move_base).
6. **יעד ב-RViz** — Publish Point (או 2D Nav Goal); הרובוט מתכנן מסלול ומגיע.
7. **איכות:** במעבר צר / ס-around אי — לא נסיעה בקו ישר דרך קיר; שימוש במפה + navfn (גלובלי) + DWA (מקומי).

Patrol / multi-waypoint **לא** ב-scope — דמו של יעד בודד.

## הרצה מהירה (מומלץ)

טרמינל אחד — הסקריפט מנהל מיפוי, hand-off, וניווט:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun summit_xl_autonomous_nav run_demo.sh
```

1. **שלב 1** — מיפוי אוטומטי (`kobuki_auto_mapping.launch` ברקע).
2. **שלב 2** — `mapping_control.py` שואל: מיפוי ידני (`y`) או סיום (`n`); שומר את המפה.
3. **שלב 3** — ניווט (`kobuki_go_to_goal.launch`).

ב-RViz: **2D Pose Estimate** (אם צריך), затן **Publish Point** על המפה.

## התקנה ב-The Construct

```bash
cd ~/catkin_ws/src
git clone https://github.com/ohadn36/autonomous-navigation-ros.git summit_xl_autonomous_nav
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

אם אין workspace:

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/ohadn36/autonomous-navigation-ros.git summit_xl_autonomous_nav
cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash
```

## הרצה שלב-שלב

### א. מיפוי אוטומטי

```bash
roslaunch summit_xl_autonomous_nav kobuki_auto_mapping.launch
```

`structured_mapper.py` — נסיעה למרכזי שטח פתוח, סיבוב 360°, `move_base` למניעת התנגשות.  
פרמטרים: `config/structured_mapper_params.yaml` (זמן מקסימלי, `corridor_zones` למטבח).

### ב. Hand-off + מיפוי ידני (אופציונלי)

```bash
rosrun summit_xl_autonomous_nav mapping_control.py
```

| מקש | פעולה |
|-----|--------|
| `i` / `,` | קדימה / אחורה |
| `j` / `l` | סיבוב |
| `k` | עצירה |
| `c` | corridor-assist (יישור במעבר צר) |

### ג. שמירת מפה

```bash
rosrun summit_xl_autonomous_nav save_map.sh
# או:
rosrun map_server map_saver -f $(rospack find summit_xl_autonomous_nav)/maps/summit_world
```

### ד. ניווט (יעד בודד)

```bash
roslaunch summit_xl_autonomous_nav kobuki_go_to_goal.launch
```

- `click_goal.py` — Publish Point → move_base; stuck recovery במעברים צרים.
- `set_initial_pose_from_odom.py` — pose ראשוני אוטומטי (ניתן לכבות: `auto_localize:=false`).

ניווט בלי click_goal (רק move_base + RViz 2D Nav Goal):

```bash
roslaunch summit_xl_autonomous_nav kobuki_navigation.launch
```

## מבנה הפרויקט

```
summit_xl_autonomous_nav/
├── launch/
│   ├── kobuki_auto_mapping.launch   # gmapping + move_base + structured_mapper
│   ├── kobuki_go_to_goal.launch     # map_server + AMCL + move_base + click_goal
│   ├── kobuki_navigation.launch     # ניווט בסיסי (ללא click_goal)
│   └── includes/gmapping.launch
├── config/
│   ├── structured_mapper_params.yaml
│   ├── click_goal_params.yaml       # stuck recovery
│   ├── kobuki_nav_costmap_common_params.yaml
│   └── kobuki_explore_*.yaml        # costmaps/planner למיפוי
├── maps/                            # summit_world.pgm + .yaml
└── scripts/
    ├── run_demo.sh                  # דמו end-to-end
    ├── structured_mapper.py
    ├── mapping_control.py
    ├── click_goal.py
    ├── exploration_utils.py
    └── save_map.sh
```

## פרמטרים חשובים

| נושא | קובץ | ברירת מחדל |
|------|------|------------|
| Laser | launch / params | `/kobuki/laser/scan` |
| זמן מיפוי מקסימלי | `structured_mapper_params.yaml` | ~200 s |
| Inflation (ניווט) | `kobuki_nav_costmap_common_params.yaml` | נמוך יותר למעברים צרים |
| Stuck recovery | `click_goal_params.yaml` | watchdog + back/align/creep |

נושא laser שונה:

```bash
roslaunch summit_xl_autonomous_nav kobuki_go_to_goal.launch scan_topic:=/your/scan
```

## פתרון תקלות

| בעיה | פתרון |
|------|--------|
| אין `/kobuki/laser/scan` | `rostopic list`; העבר `scan_topic:=...` ב-launch |
| מפה ריקה ב-RViz | Fixed Frame = `map`; המתן ל-gmapping / AMCL |
| חלקיקי AMCL מפוזרים | **2D Pose Estimate** ליד מיקום הרובוט |
| הרובוט לא זז | `rostopic echo /cmd_vel`; `rostopic echo /move_base/status` |
| תקוע במעבר צר | recovery אוטומטי ב-`click_goal`; במיפוי ידני — מקש `c` |
| `launch_hint.py` / build ישן | `git pull && catkin_make && source devel/setup.bash` |

## מסמכי קורס

- [ ] מצגת (10–12 שקפים) + קישור GitHub וסרטון
- [ ] מסמך תיאוריה (5–7 עמודים)
- [ ] הקלטת דמו (5–12 דק') — מיפוי + ניווט + עקיפת מכשול
- [ ] מאגר GitHub זה
- [ ] טופס Project Summary

## מקורות

- [ROS Navigation in 5 Days — The Construct](https://app.theconstructsim.com/desktop/course/57)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)

## רישיון

MIT — שימוש אקדמי בפרויקט הקורס.
