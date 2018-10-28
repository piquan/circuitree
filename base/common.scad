slot_width=16;
slot_depth=2;
outer_wall=1;
base_height=13;

webbing_cover=40;
webbing_width=0.1;
webbing_dist=2;

module webbing_base() {
     for (i = [0 : webbing_dist : webbing_cover]) {
          translate([i - webbing_width/2, 0, 0]) {
               cube([webbing_width, webbing_cover, base_height]);
          }
          translate([0, i - webbing_width/2, 0]) {
               cube([webbing_cover, webbing_width, base_height]);
          }
     }
}

module webbing() {
     rotate([0, 0, 45]) {
          translate([-webbing_cover/2, -webbing_cover/2, 0]) {
               webbing_base();
          }
     }
}

module slot() {
     translate([0, 0, base_height/2]) {
          cube([slot_width, slot_depth,
                base_height*2], center=true);
     }
}

module slot_wall() {
     translate([0, 0, outer_wall*5]) {
          cube([slot_width+(outer_wall*2), slot_depth+(outer_wall*2),
                base_height+(outer_wall*3)], center=true);
     }
}

module slot_midwall() {
     translate([0, 0, outer_wall*2]) {
          cube([slot_width+outer_wall, slot_depth+outer_wall,
                base_height+outer_wall], center=true);
     }
}

module slot_base() {
     for (t = [slot_width/4, -slot_width/4]) {
          translate([t, 0, outer_wall/2]) {
               cube([slot_depth, slot_depth, outer_wall], center=true);
          }
     }
}

