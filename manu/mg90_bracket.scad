

module mg_90(){
	color("grey"){

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
	}


	color("red"){

	translate([13.5,0,0])
	rotate([90,0,0])
	cylinder(r=1.2,h=15,center=true,$fn=20);


	translate([-13.5,0,0])
	rotate([90,0,0])
	cylinder(r=1.2,h=15,center=true,$fn=20);
	}
}


cl=0.8;

module base_bracket(){
	difference(){

		cube(size=[35,10,16], center=true);
		translate([0,-2,0])
		cube(size=[32+cl,12,12+cl], center=true);

		mg_90();
	}
}


module servo(){
	mg_90();

	color("white")
	translate([5,-10,0])
	rotate([-90,0,0])
	rotate([0,0,90])
	import("horns/SG90_four_horns_with_holes.stl");
}

module shoulder_joint(left = 0){
	

	translate([0,0,5])
	rotate([0,90,0]){

		base_bracket();
		servo();
	}

	// symm_z = 0;

	// if(left)
	// {
	// 	symm_z = 90;
	// }
	if(left){
		translate([-5,-22,-37])
		rotate([0,90,0])
		rotate([90,0,0])
		{
			rotate([0,180,180])
			{

				base_bracket();
				servo();
			}
			

			translate([-18,6,0])
			cube([2,36,30],center=true);		
		}
	}
	else{
		translate([5,-22,-37])
		rotate([0,90,0])
		rotate([90,0,0])
		{

			base_bracket();
			servo();
			

			translate([-18,-6,0])
			cube([2,36,30],center=true);		
		}

	}
	if(left){
		translate([0,0,-5])
		rotate([0,180,0])
		{

			translate([5,-22,0])
			rotate([90,0,0]){

			translate([0,-10,-7])
			// difference(){

				cube(size=[30,30,2], center=true);
				// translate([0,0,0])
				// cube(size=[20+cl,12,12+cl], center=true);
				// }

				// rotate([180,0,180])
				rotate([0,0,180])
				{

				base_bracket();
				servo();
				}
			}
		}	
	}else{
		translate([10,0,-5])
		rotate([0,180,0])
		{

			translate([5,-22,0])
			rotate([90,0,0]){

			translate([0,-10,-7])
			// difference(){

				cube(size=[30,30,2], center=true);
				// translate([0,0,0])
				// cube(size=[20+cl,12,12+cl], center=true);
				// }

				rotate([180,0,180])
				rotate([0,0,180])
				{

				base_bracket();
				servo();
				}
			}
		}		
	}
}



// shoulder_joint();


// translate([0,0,5])
// rotate([0,90,0])
// difference(){
// 	hull(){
// 		translate([0,30,0])
// 		cube([50,2,50],center=true);
// 		base_bracket();
// 	}
// 	translate([0,15,0])
// 	cube([34,25,70],center=true);
// 	translate([0,15,0])
// 	cube([34,55,14],center=true);
// }



// translate([0,150,0])
// rotate([0,0,180])
// {

// shoulder_joint(2);
// translate([0,0,5])
// rotate([0,90,0])
// difference(){
// 	hull(){
// 		translate([0,30,0])
// 		cube([50,2,50],center=true);
// 		base_bracket();
// 	}
// 	translate([0,15,0])
// 	cube([34,25,70],center=true);
// 	translate([0,15,0])
// 	cube([34,55,14],center=true);
// }

// }



// translate([-6,-22,-100])
// cube([4,20,150],center=true);


// translate([-5,-22,-37])
// rotate([0,90,0])
// rotate([90,0,0])
// {
// 	rotate([0,180,180])
// 	{

// 		base_bracket();
// 		servo();
// 	}
	

// 	translate([-18,6,0])
// 	cube([2,36,30],center=true);		
// }


// translate([-6,172,-48])
// cube([4,20,50],center=true);




// translate([-15,172,-65])
// rotate([-90,180,0])
// shoulder_joint();






// translate([-15,0,-65])
// rotate([-90,180,180])
// shoulder_joint();
