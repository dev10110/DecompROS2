#include <decomp_util/line_segment.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_geometry/geometric_utils.h>


int main(int argc, char **argv) {

  // Create obstacles
  int N_obs = 100;
  vec_Vec3f obs;
  for (int i=0; i<N_obs; i++)
  {

    double x = 2 * ((double)rand()) / INT_MAX - 1;
    double y = 2 * ((double)rand()) / INT_MAX - 1;
    double z = 2 * ((double)rand()) / INT_MAX- 1 ;

    double n = sqrt(x*x + y*y + z*z);

    obs.push_back({10.0 * x/n, 10.0 * y/n, 10.0* z/n});
  }


  // Seed
  const Vec3f pos1(0.0, 0.0, 1);
  const Vec3f pos2(0.1, 0.0, 1);

  // Initialize SeedDecomp3D
  LineSegment3D decomp(pos1, pos2);
  //SeedDecomp3D decomp(pos1);
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec3f(1, 1, 1));
  decomp.dilate(0);
  
  const auto poly = decomp.get_polyhedron();
  const auto faces = cal_vertices(poly);

  std::cout << "found " << faces.size() << " faces" << std::endl;

  for (auto f: faces)
  {
    std::cout << "FACE " << std::endl;
    for (auto v: f)
    {
      std::cout << v(0) <<  " " << v(1) << " " << v(2) << std::endl ;
    }
}


  return 0;
}
