include <common.scad>;
include <qp.scad>;

module outer() {
     quarter_outer();
     mirror([1,0,0]) quarter_outer();
     mirror([0,1,0]) quarter_outer();
     mirror([1,1,0]) quarter_outer();
}

module inner() {
     quarter_inner();
     mirror([1,0,0]) quarter_inner();
     mirror([0,1,0]) quarter_inner();
     mirror([1,1,0]) quarter_inner();
}

module shell() {
     outer();
}

module main() {
     intersection() {
          outer();
          union() {
               difference() {
                    union() {
                         shell();
                         slot_wall();
                    }
                    slot();
               }
               slot_base();
          }
     }
}

main();
