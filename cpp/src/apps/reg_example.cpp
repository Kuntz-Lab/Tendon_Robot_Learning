#include "mag-tracker/svd_reg.h"
#include "cliparser/CliParser.h"
#include "cpptoml/toml_conversions.h"

void populate_parser(CliParser &parser){
    parser.set_program_description(
                "An example app that takes 2 point clouds from 2 csv files and outputs\n"
                "the rotation and translation components of the transform\n"
                "and outputs to provided toml file");
    parser.add_positional("point_cloud_1");
    parser.set_required("point_cloud_1");
    parser.add_positional("point_cloud_2");
    parser.set_required("point_cloud_2");
    parser.add_argflag("-o","-output");
}
int main(int arg_count, char* arg_list[]){
    CliParser parser;
    populate_parser(parser);
    parser.parse(arg_count, arg_list);
    SVD_Reg sreg;
    auto p1s= sreg.load_point_cloud(parser["point_cloud_1"]);
    auto p2s= sreg.load_point_cloud(parser["point_cloud_2"]);
    std::cout << "computing the rigid transformations...\n";
    sreg.computeRigidTransform(p1s, p2s);
    auto t_points=sreg.compute_transformed_points(p1s);
    std::cout <<"Rotation matrix is\n"<< sreg.rot_matrix() << endl;
    std::cout <<"Translation vector is\n"<< sreg.translation() << endl;
    std::cout << endl;
    if(parser.has("-o")){
        cpptoml::to_file(sreg,parser["-o"]);
    }

    return 0;
}
