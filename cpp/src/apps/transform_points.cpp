#include "mag-tracker/svd_reg.h"
#include "cliparser/CliParser.h"
#include <QCoreApplication>
#include <QDebug>
void populate_parser(CliParser &parser){
    parser.set_program_description(
                "An example app that takes a point cloud from a csv file and outputs\n"
                "the transformed points");
    parser.add_positional("point_cloud");
    parser.set_required("point_cloud");
    parser.add_positional("transform");
    parser.set_required("transform");
}

int main(int arg_count,char* arg_list[]){
    CliParser parser;
    populate_parser(parser);
    parser.parse(arg_count,arg_list);
    auto RT=cpptoml::from_file<SVD_Reg>(parser["transform"]);
    auto p1s= RT.load_point_cloud(parser["point_cloud"]);
    auto p2s= RT.compute_transformed_points(p1s);
    for(auto& pt:p2s){
        std::cout<<pt(0)<<" "<<pt(1)<<" "<<pt(2)<<std::endl;
    }


}
