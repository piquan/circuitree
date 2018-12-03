// You can set a name to have a personalized name plaque.  You can also
// set this on the command line with '-Dname="foo"', which will override
// this value.
name="";
bxr=1;                          // Box rounding

module gift(dx, dy, dz, x, y, z, tx, ty, tz, ribbonside=3) {
     // Prevent voids under boxes by extending them to the table.
     // Where this causes visible artifacts, it's best to make the box
     // taller.
     // This is the convex hull formed by (1) the box's bottom surface
     // and (2) the box's bottom surface projected on the xy plane.
     hull() {
          linear_extrude(height=0.1) projection()
               translate([x, y, z+(dz/2)-0.5]) rotate([tx, ty, tz]) hull() {
               translate([dx/2-bxr, dy/2-bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([dx/2-bxr, -dy/2+bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, dy/2-bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, -dy/2+bxr, -dz/2+bxr]) sphere(r=bxr);
          }
          translate([x, y, z+(dz/2)-0.5]) rotate([tx, ty, tz]) hull() {
               translate([dx/2-bxr, dy/2-bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([dx/2-bxr, -dy/2+bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, dy/2-bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, -dy/2+bxr, -dz/2+bxr]) sphere(r=bxr);
          }
     }
     translate([x, y, z+(dz/2)-0.5]) rotate([tx, ty, tz]) {
          // Box
          //cube([dx, dy, dz], center=true);
          // We actually use the hull of spheres for the box to prevent
          // lots of sharp 90deg corners.
          hull() {
               translate([dx/2-bxr, dy/2-bxr, dz/2-bxr]) sphere(r=bxr);
               translate([dx/2-bxr, dy/2-bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([dx/2-bxr, -dy/2+bxr, dz/2-bxr]) sphere(r=bxr);
               translate([dx/2-bxr, -dy/2+bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, dy/2-bxr, dz/2-bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, dy/2-bxr, -dz/2+bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, -dy/2+bxr, dz/2-bxr]) sphere(r=bxr);
               translate([-dx/2+bxr, -dy/2+bxr, -dz/2+bxr]) sphere(r=bxr);
          }
          // Bow body
          translate([0, 0, dz/2+.25]) hull() {
               translate([-.5, -.5, 0]) sphere(r=0.5);
               translate([-.5, .5, 0]) sphere(r=0.5);
               translate([.5, -.5, 0]) sphere(r=0.5);
               translate([.5, .5, 0]) sphere(r=0.5);
          }
          // Bow ribbons
          bowrot = (ribbonside >= 4 ? 90 : 0);
          rotate([0, 0, bowrot]) translate([0, 0, dz/2]) hull() {
               translate([-2, -2, 0.5]) sphere(r=0.25);
               translate([-2, 2, 0.5]) sphere(r=0.25);
               translate([-2, -2, 0]) sphere(r=0.25);
               translate([-2, 2, 0]) sphere(r=0.25);
               sphere(r=0.5);
          }
          rotate([0, 0, bowrot]) translate([0, 0, dz/2]) hull() {
               translate([2, -2, 0.25]) sphere(r=0.25);
               translate([2, 2, 0.25]) sphere(r=0.25);
               translate([2, -2, 0]) sphere(r=0.25);
               translate([2, 2, 0]) sphere(r=0.25);
               sphere(r=0.5);
          }
          // Box ribbons
          if (ribbonside == 1 || ribbonside == 3 ||
              ribbonside == 5 || ribbonside == 7) {
               hull() {
                    translate([dx/2, .5, dz/2]) sphere(r=0.25);
                    translate([dx/2, -.5, dz/2]) sphere(r=0.25);
                    translate([-dx/2, .5, dz/2]) sphere(r=0.25);
                    translate([-dx/2, -.5, dz/2]) sphere(r=0.25);
                    translate([dx/2, .5, -dz/2]) sphere(r=0.25);
                    translate([dx/2, -.5, -dz/2]) sphere(r=0.25);
                    translate([-dx/2, .5, -dz/2]) sphere(r=0.25);
                    translate([-dx/2, -.5, -dz/2]) sphere(r=0.25);
               }
          }
          if (ribbonside == 2 || ribbonside == 3 ||
              ribbonside == 6 || ribbonside == 7) {
               hull() {
                    translate([.5, dy/2, dz/2]) sphere(r=0.25);
                    translate([.5, -dy/2, dz/2]) sphere(r=0.25);
                    translate([-.5, dy/2, dz/2]) sphere(r=0.25);
                    translate([-.5, -dy/2, dz/2]) sphere(r=0.25);
                    translate([.5, dy/2, -dz/2]) sphere(r=0.25);
                    translate([.5, -dy/2, -dz/2]) sphere(r=0.25);
                    translate([-.5, dy/2, -dz/2]) sphere(r=0.25);
                    translate([-.5, -dy/2, -dz/2]) sphere(r=0.25);
               }
          }
     }
}

// This is an approximation of the desired shape.  Use it with % for debugging.
module approximation() {
     intersection() {
          // This sphere is designed so that it's 100mm at the z=0
          // plane, and 10mm high.  Solving
          // (100/2)^2 + (r-10)^2 = r^2 gives r=130.
          translate([0, 0, -120]) sphere(r=130);
          // Cut it off at the table.
          translate([-51, -51, 0]) cube([102, 102, 20], center=false);
     }
}
//%approximation();

module smallest() {
     // This is a small section to make sure that there aren't
     // critical gaps in the area that prevent it from functioning as
     // a support base.
     intersection() {
          // This sphere is designed so that it's 80mm at the z=0
          // plane, and 8mm high.  Solving
          // (80/2)^2 + (r-8)^2 = r^2 gives r=104.
          translate([0, 0, -96]) sphere(r=104);
          // Cut it off at the table.
          translate([-51, -51, 0]) cube([102, 102, 20], center=false);
     }
}

// Negative elements
module keepout() {
     // A bit of clearance for the board's components etc
     translate([-9, -4, 10]) cube([18, 5, 10], center=false);
}
module slot() {
     // Slot for the board to sit in
     translate([0, 0, 8]) cube([27, 2, 20], center=true);
}
module table() {
     // Flat bottom for the table or other surface
     translate([-100, -100, -50]) cube([200, 200, 50], center=false);
}
module usbchan() {
     // Channel for the USB connector.  Assume the USB shroud goes 2mm
     // on either side of the connector's footprint edge.  The
     // connector is 14mm from the bottom of the board, which is
     // 2mm from the table (see slot_struts).  That means the channel
     // starts 14mm from the table.
     // The connector is 9mm wide, so with margins use 13mm.
     translate([10, -4, 14]) {
          cube([60, 8, 13]);
     }
}
%usbchan();

module slot_struts() {
     // Small struts along the bottom for the board to sit on
     // If you move these, fix usbchan() accordingly.
     translate([-5, 0, 1]) cube([2, 3, 2], center=true);
     translate([5, 0, 1]) cube([2, 3, 2], center=true);
}

module gifts() {
     scale(1.2) {
          //   SIZE                POSITION            ROTATION
          gift(21, 19, 5,          10, 10, 2.5,        5, 10, 15,     0);
          gift(19, 20, 6,          8, -11, 3,          10, 5, 0,      5);
          gift(13, 8, 5,           -13, 5, 4,          -5, -10, 15,   2);
          gift(12, 12, 5,          -8, 17, 4,          -5, -5, -5,    8);
          gift(17, 17, 6,          -10, -12, 5,        10, -15, -12,  3);
          gift(8, 12, 8,           2, 18, 4,           -20, -15, 10,  6);
          gift(24, 6, 3,           20, 1, 4,           -20, 15, 10,   2);
          gift(15, 20, 2,          -25, -8, 3,         15, -15, 5,    3);
          gift(16, 19, 7,          -22, 10, 2.5,       -15, -15, 5,   4);
          gift(21, 17, 8,          -20, -25, -1,        15, -10, -35,  6);
          gift(10, 25, 6,          -35, 0, 1,          0, -20, 10,    2);
          gift(12, 20, 3,          30, -15, 2.5,       5, 10, -20,    8);
          gift(15, 15, 5,          20, -10, 6,         5, 10, 60,     0);
          gift(20, 20, 3,          13, -22, 6,         10, 10, 10,    6);
          gift(30, 10, 7,          15, -35, 0,         15, -5, 15,    2);
          gift(15, 25, 6,          -5, -30, 2,         15, -5, -20,   8);
          gift(20, 10, 4,          -5, 10, 10,         10, -5, -45,   3);
          gift(20, 10, 3,          30, 15, 2,          -20, 5, -45,   5);
          gift(11, 16, 4,          25, 12, 5,          0, 15, 35,     2);
          gift(11, 26, 6,          12, 24, 3,          -20, 5, -5,    8);
          gift(14, 14, 6,          34, 0, 1,           -10, 25, 0,    0);
          gift(24, 8,  8,          0, 34, 1,           -25, -10, 0,   5);
          gift(17, 13, 4,          0, 24, 5,           -20, -5, 10,   2);
          gift(19, 20, 6,          -19, 22, 3,         -35, -5, 40,   8);
          gift(15, 15, 8,          25, 28, 0,          0, 0, 45,      4);
          gift(10, 15, 8,          -32, -16, 0,        0, 0, 30,      4);
          gift(15, 10, 8,          -32, 20, 0,         0, 0, 30,      2);
     }
}

plx=50;
module plaque() {
     hull() {
          translate([-plx, -20, 0]) sphere(r=bxr);
          translate([plx, -20, 0]) sphere(r=bxr);
          translate([-plx, -40, 0]) sphere(r=bxr);
          translate([plx, -40, 0]) sphere(r=bxr);
          translate([-plx, -20, 15]) sphere(r=bxr);
          translate([plx, -20, 15]) sphere(r=bxr);
          translate([-plx, -40, 10]) sphere(r=bxr);
          translate([plx, -40, 10]) sphere(r=bxr);
     }
     translate([0, -35, 10]) rotate([15, 0, 0])
          linear_extrude(height=3)
          // The position of the name is 1mm different if there are
          // descender letters.  Unfortunately, failed searches
          // generate a warning.
          translate([0, search("gjpqy", name) ? -.5 : -1.5])
          text(name, size=15, halign="center");
}
module plaque_keepout() {
     translate([-plx, -40, 0]) cube([2*plx, 20, 30], center=false);
}

module main() {
  difference() {
    union() {
         // Positive elements
         #smallest();
         difference() {
              gifts();
              if (name) plaque_keepout();
         }
         if (name) plaque();
    }
    // Negative elements
    keepout();
    slot();
    table();
    usbchan();            // Currently unneeded
  }
  // "Overriding" positive elements
  slot_struts();
}
main();
