// PCB width: 15mm
// PCB slot height: 12mm
// PCB thickness: 1.6mm

include <common.scad>;
box_width=25;
box_depth=25;
box_height=10;

module base_hull() {
     cube(size=[box_width, box_depth, box_height]);
}

module hull() {
     minkowski() {
          base_hull();
          intersection() {
               sphere(r=outer_wall, $fn=20);
               translate([-outer_wall, -outer_wall, 0]) {
                    cube([outer_wall*2, outer_wall*2, outer_wall]);
               }
          }
     }
}

module main() {
     difference() {
          union() {
               hull();
               intersection() {
                    translate([box_width/2, box_depth/2, 0]) slot_wall();
                    base_hull();
               }
          }
          translate([box_width/2, box_depth/2, 0]) slot();
     }
     translate([box_width/2, box_depth/2, 0]) slot_base();
}
main();
