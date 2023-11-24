

module mg_90(){
	scale([32,3,12])
	cube(1,center=true);


	translate([0,6.5,0])
	scale([23,25,12])
	cube(1,center=true);


	translate([5,0,0])
	rotate([90,0,0])
	cylinder(r=6,h=8,center=false);


	translate([5,0,0])
	rotate([90,0,0])
	cylinder(r=2.5,h=12,center=false);



	translate([13.5,0,0])
	rotate([90,0,0])
	cylinder(r=1.2,h=15,center=true,$fn=20);


	translate([-13.5,0,0])
	rotate([90,0,0])
	cylinder(r=1.2,h=15,center=true,$fn=20);
}


cl=0.8;

// hull()
difference(){

cube(size=[35,10,16], center=true);
translate([0,-2,0])
cube(size=[32+cl,12,12+cl], center=true);

mg_90();
}