/******************************************************************************/
/* Handy Functions                                                            */
/******************************************************************************/

// Aliases for translate
module mov_x(d)            { translate(v=[ d,  0,  0]) children(); }
module mov_y(d)            { translate(v=[ 0,  d,  0]) children(); }
module mov_z(d)            { translate(v=[ 0,  0,  d]) children(); }
module mov_xy(xd, yd)      { translate(v=[xd, yd,  0]) children(); }
module mov_xz(xd, zd)      { translate(v=[xd,  0, zd]) children(); }
module mov_yz(yd, zd)      { translate(v=[ 0, yd, zd]) children(); }
module mov_xyz(xd, yd, zd) { translate(v=[xd, yd, zd]) children(); }

// Aliases for rotate
module rot_x(a)            { rotate(a=[ a,  0,  0]) children(); }
module rot_y(a)            { rotate(a=[ 0,  a,  0]) children(); }
module rot_z(a)            { rotate(a=[ 0,  0,  a]) children(); }
module rot_xy(xa, ya)      { rotate(a=[xa, ya,  0]) children(); }
module rot_xz(xa, za)      { rotate(a=[xa,  0, za]) children(); }
module rot_yz(ya, za)      { rotate(a=[ 0, ya, za]) children(); }
module rot_xyz(xa, ya, za) { rotate(a=[xa, ya, za]) children(); }

// Duplicate an object moving one positive and one negative
module dupe_x(d) { mov_x(d) children(); mov_x(-d) children(); }
module dupe_y(d) { mov_y(d) children(); mov_y(-d) children(); }
module dupe_z(d) { mov_z(d) children(); mov_z(-d) children(); }

// Mirror while duplicating
module mir_x() { children(); mirror(v=[1, 0, 0]) children(); }
module mir_y() { children(); mirror(v=[0, 1, 0]) children(); }
module mir_z() { children(); mirror(v=[0, 0, 1]) children(); }

// Move and mirror while duplicating
module mov_mir_x(d) { mir_x() mov_x(d) children(); }
module mov_mir_y(d) { mir_y() mov_y(d) children(); }
module mov_mir_z(d) { mir_z() mov_z(d) children(); }

// Rotate while duplicating
module rot_dupe_x(a, i) { for (j=[1:i]) { rot_x(a*j) children(); } }
module rot_dupe_y(a, i) { for (j=[1:i]) { rot_y(a*j) children(); } }
module rot_dupe_z(a, i) { for (j=[1:i]) { rot_z(a*j) children(); } }

// cube_[pcn][pcn][pcn] is aligned [p]ositive [c]enter or [n]egative for x y z
module cube_ppp(x, y, z) { mov_xyz(   0,    0,    0) cube(size=[x, y, z]); }
module cube_ppc(x, y, z) { mov_xyz(   0,    0, -z/2) cube(size=[x, y, z]); }
module cube_ppn(x, y, z) { mov_xyz(   0,    0,   -z) cube(size=[x, y, z]); }

module cube_pcp(x, y, z) { mov_xyz(   0, -y/2,    0) cube(size=[x, y, z]); }
module cube_pcc(x, y, z) { mov_xyz(   0, -y/2, -z/2) cube(size=[x, y, z]); }
module cube_pcn(x, y, z) { mov_xyz(   0, -y/2,   -z) cube(size=[x, y, z]); }

module cube_pnp(x, y, z) { mov_xyz(   0,   -y,    0) cube(size=[x, y, z]); }
module cube_pnc(x, y, z) { mov_xyz(   0,   -y, -z/2) cube(size=[x, y, z]); }
module cube_pnn(x, y, z) { mov_xyz(   0,   -y,   -z) cube(size=[x, y, z]); }


module cube_cpp(x, y, z) { mov_xyz(-x/2,    0,    0) cube(size=[x, y, z]); }
module cube_cpc(x, y, z) { mov_xyz(-x/2,    0, -z/2) cube(size=[x, y, z]); }
module cube_cpn(x, y, z) { mov_xyz(-x/2,    0,   -z) cube(size=[x, y, z]); }

module cube_ccp(x, y, z) { mov_xyz(-x/2, -y/2,    0) cube(size=[x, y, z]); }
module cube_ccc(x, y, z) { mov_xyz(-x/2, -y/2, -z/2) cube(size=[x, y, z]); }
module cube_ccn(x, y, z) { mov_xyz(-x/2, -y/2,   -z) cube(size=[x, y, z]); }

module cube_cnp(x, y, z) { mov_xyz(-x/2,   -y,    0) cube(size=[x, y, z]); }
module cube_cnc(x, y, z) { mov_xyz(-x/2,   -y, -z/2) cube(size=[x, y, z]); }
module cube_cnn(x, y, z) { mov_xyz(-x/2,   -y,   -z) cube(size=[x, y, z]); }


module cube_npp(x, y, z) { mov_xyz(  -x,    0,    0) cube(size=[x, y, z]); }
module cube_npc(x, y, z) { mov_xyz(  -x,    0, -z/2) cube(size=[x, y, z]); }
module cube_npn(x, y, z) { mov_xyz(  -x,    0,   -z) cube(size=[x, y, z]); }

module cube_ncp(x, y, z) { mov_xyz(  -x, -y/2,    0) cube(size=[x, y, z]); }
module cube_ncc(x, y, z) { mov_xyz(  -x, -y/2, -z/2) cube(size=[x, y, z]); }
module cube_ncn(x, y, z) { mov_xyz(  -x, -y/2,   -z) cube(size=[x, y, z]); }

module cube_nnp(x, y, z) { mov_xyz(  -x,   -y,    0) cube(size=[x, y, z]); }
module cube_nnc(x, y, z) { mov_xyz(  -x,   -y, -z/2) cube(size=[x, y, z]); }
module cube_nnn(x, y, z) { mov_xyz(  -x,   -y,   -z) cube(size=[x, y, z]); }

// square_[pcn][pcn] is aligned [p]ositive [c]enter or [n]egative for x y
module square_pp(x, y) { mov_xy(   0,    0) square(size=[x, y]); }
module square_pc(x, y) { mov_xy(   0, -y/2) square(size=[x, y]); }
module square_pn(x, y) { mov_xy(   0,   -y) square(size=[x, y]); }

module square_cp(x, y) { mov_xy(-x/2,    0) square(size=[x, y]); }
module square_cc(x, y) { mov_xy(-x/2, -y/2) square(size=[x, y]); }
module square_cn(x, y) { mov_xy(-x/2,   -y) square(size=[x, y]); }

module square_np(x, y) { mov_xy(  -x,    0) square(size=[x, y]); }
module square_nc(x, y) { mov_xy(  -x, -y/2) square(size=[x, y]); }
module square_nn(x, y) { mov_xy(  -x,   -y) square(size=[x, y]); }

// Used to fix render bugs on preview by making holes slightly longer than needed
eps = 0.04;
module nudge_x() { mov_x(-eps) children(); }
module nudge_y() { mov_y(-eps) children(); }
module nudge_z() { mov_z(-eps) children(); }

module poke_x() { mov_x(eps) children(); }
module poke_y() { mov_y(eps) children(); }
module poke_z() { mov_z(eps) children(); }

// see
// https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/undersized_circular_objects
// extracting the fudge factor
function outer_radius(r, fn)    = r/cos(180/fn);
function center_radius(r, fn)   = r * (1 + 1/cos(180/fn))/2;
function outer_diameter(d, fn)  = outer_radius(d, fn);
function center_diameter(d, fn) = center_radius(d, fn);
module cylinder_outer(h, r, fn) {
  cylinder(h=h, r=outer_radius(r, fn), $fn=fn);
}
module cylinder_center(h, r, fn) {
  cylinder(h=h, r=center_radius(r, fn), $fn=fn);
}

// So that circles can be aligned to xy
function round8(n) = ceil(round(n)/8)*8;
function round4(n) = ceil(round(n)/4)*4;
function get_fn_d(d, s=10) = round4(PI*d/s);
function get_fn_r(r, s=10) = get_fn_d(2*r, s);

// given a 'dict` returns the value associated with the given key
// the dict is a list of the form [(key0, value0), (key1, value1), ... ]
function lookup(d, k) = d[search([k],[for(i=[0:2:len(d)-2])d[i]])[0]*2+1];

// concatenate two lists together
function cat(L1, L2) = [for(L=[L1, L2], a=L) a];

module spline(p0, p1, p2, p3, fn, debug=false) {
  x0 = p0[0]; y0 = p0[1];   x1 = p1[0]; y1 = p1[1];
  x2 = p2[0]; y2 = p2[1];   x3 = p3[0]; y3 = p3[1];

  start = [[x3, 0], [x0, 0]];

  ts  = [for(i=[0:fn]) i/fn];
  tms = [for(i=[0:fn]) 1.0 - ts[i]];

  as  = [for(i=[0:fn]) pow(tms[i], 3)];
  bs  = [for(i=[0:fn]) 3.0 * ts[i] * pow(tms[i], 2)];
  cs  = [for(i=[0:fn]) 3.0 * pow(ts[i], 2) * tms[i]];
  ds  = [for(i=[0:fn]) pow(ts[i], 3)];

  xs  = [for(i=[0:fn]) as[i] * x0 + bs[i] * x1 + cs[i] * x2 + ds[i] * x3];
  ys  = [for(i=[0:fn]) as[i] * y0 + bs[i] * y1 + cs[i] * y2 + ds[i] * y3];

  body = [for(i=[0:fn]) [xs[i], ys[i]]];

  points = cat(start, body);
  polygon(points=points);

  if (debug) {
    color("red")   mov_xy(x0, y0) cube_xy(2,2,2);
    color("green") mov_xy(x1, y1) cube_xy(2,2,2);
    color("green") mov_xy(x2, y2) cube_xy(2,2,2);
    color("red")   mov_xy(x3, y3) cube_xy(2,2,2);
  }
}

module stripes(stripe_x, space_x, stripe_y, stripe_z, stripes) {
  unit_width = stripe_width + space_width;
  total_width = unit_width*stripes - space_width;
  mov_y(-total_width/2) {
    linear_extrude(height=stripe_z) {
      for (i=[0:stripes-1]) {
        mov_x(i*unit_width)
          square_pc(stripe_width, total_y);
      }
    }
  }
}

module 2d_hex_grid(rows, cols, r, z, s) {
  half_rows = floor(rows / 2);
  odd_cols = cols % 2;
  even_cols = abs(odd_cols - 1);
  half_cols = floor(cols / 2) - even_cols;

  d_x = 2*r + s;
  d_y = 3*(r + s/2)/sqrt(3);


  linear_extrude(height=z) {
    for (y=[-half_rows:half_rows]) {
      y_odd = abs(y) % 2;
      lower = -(half_cols + even_cols*y_odd);
      upper = half_cols - odd_cols*y_odd;
      for (x=[lower:upper]) {
        mov_xy(y_odd* d_x/2 + x*d_x, y*d_y)
          rot_z(180/6)
          circle(r=outer_radius(r, 6), $fn=6);
      }
    }
  }
}

// Based on
// https://www.thingiverse.com/thing:370871
module fibonacci_sphere(r, n)
{
  golden_ratio = 1.618033988749894848204586834365638117720309179805762862135;
  golden_angle = 360 * golden_ratio;
  np = 2*n - 1;

  function lon(i) = golden_angle*i;
  function xy(z) = sqrt(1 - pow(z,2));
  function z(i,n) = 2*i / (2*n + 1);
  function pos(i, n) = [cos(lon(i)) * xy(z(i,n)),
                        sin(lon(i)) * xy(z(i,n)),
                        z(i,n)];

  hull()
    polyhedron(points=[for(i=[-n:(n - 2)]) r * pos(i,n)],
               faces=[for(i=[0:3:2*n]) [i%np, (i + 1)%np, (i + 2)%np]]);
}

// Converts from inches to mm
function mm(in) = (in*25.4);

// Splits the time variable
function split_time(low, high) = low + (high - low)*$t;
