// Pybadge VL53L5CX Mount
//
// This can be glued into the existing Pybadge camera 3D print
// to mount a a VL53L5CX depth sensor.
//
// Daniel Winker, June 26, 2023

standoffHeight = 10;
innerWidth = 28;
outerWidth = 35;
frameThickness = 2;
legLength = 10;
thickness = 3.5;
legWidth = (outerWidth - innerWidth)/2;

screwDia = 2.3;
screwSep = 20.32;
screwHeight = 8;

difference() {
    // The main body
    union() {
        translate([0, 0, frameThickness]) {
            cube([outerWidth, thickness, standoffHeight]);   
        }
        translate([legWidth, 0, 0]) {
            cube([innerWidth, thickness, frameThickness]);   
        }
    }
    // The screw holes
    translate([outerWidth/2-screwSep/2, thickness/2, standoffHeight+frameThickness-screwHeight]) {  
        cylinder(r=screwDia/2, screwHeight, $fn=20);
    }
    translate([outerWidth/2+screwSep/2, thickness/2, standoffHeight+frameThickness-screwHeight]) {  
        cylinder(r=screwDia/2, screwHeight, $fn=20);
    }
}

// The legs
// I made the legs as tall as the standoffs so this can be printed upside-down
// without supports. This results in the smoothest possible bottom-surface
// for the legs.
translate([0, -legLength/2+thickness/2, frameThickness]) {
    cube([legWidth, legLength, standoffHeight]);
}
translate([outerWidth-legWidth, -legLength/2+thickness/2, frameThickness]) {
    cube([legWidth, legLength, standoffHeight]);
}