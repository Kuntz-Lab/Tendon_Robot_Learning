#ifndef svd_reg_h
#define svd_reg_h
#include "csv/Csv.h"
#include "util/openfile_check.h"
#include<fstream>
#include <vector>
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <memory>
#include <cpptoml/toml_conversions.h>
#include <cpptoml/cpptoml.h>
#include <QVector3D>
#include <QVector>


using namespace Eigen;
using namespace std;

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> TransformType;
typedef std::vector<Eigen::Vector3d>                PointsType;
namespace cpptoml { class table; }


class SVD_Reg
{
public:

    std::shared_ptr<cpptoml::table> to_toml() const {


        auto tbl = cpptoml::make_table();
        auto trans = cpptoml::make_array();
        tbl->insert("rotation",cpptoml::to_toml(this->_R));
        tbl->insert("translation", trans);
        trans->push_back(this->_T(0));
        trans->push_back(this->_T(1));
        trans->push_back(this->_T(2));
        return tbl;
    }
    static SVD_Reg from_toml(std::shared_ptr<cpptoml::table> tbl) {


        SVD_Reg transform_info;


        auto rot_tbl = tbl->get("rotation")->as_table();



        auto trans = tbl->get("translation")->as_array();

        auto point = cpptoml::to_point(trans);
        transform_info._R=cpptoml::to_matrix(rot_tbl);
        transform_info._T=point;


        return transform_info;
    }
    void computeRigidTransform(const PointsType& src, const PointsType& dst)
    {
        assert(src.size() == dst.size());
        int pairSize = src.size();
        Eigen::Vector3d center_src(0, 0, 0), center_dst(0, 0, 0);
        for (int i=0; i<pairSize; ++i)
        {
            center_src += src[i];
            center_dst += dst[i];
        }
        center_src /= (double)pairSize;
        center_dst /= (double)pairSize;

        Eigen::MatrixXd S(pairSize, 3), D(pairSize, 3);
        for (int i=0; i<pairSize; ++i)
        {
            for (int j=0; j<3; ++j)
                S(i, j) = src[i][j] - center_src[j];
            for (int j=0; j<3; ++j)
                D(i, j) = dst[i][j] - center_dst[j];
        }
        Eigen::MatrixXd Dt = D.transpose();
        Eigen::Matrix3d H = Dt*S;

        BDCSVD<Eigen::MatrixXd> svd;
        Eigen::Matrix3d H_;

        H_=H;
        svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV );
        if (!svd.computeU() || !svd.computeV()) {
            std::cerr << "decomposition error" << endl;
            _R=Eigen::Matrix3d::Identity();
            _T=Eigen::Vector3d::Zero();
            return;

        }
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Matrix3d Vt = V.transpose();
        Eigen::Matrix3d R = svd.matrixU()*Vt;
        if(R.determinant()<0){
            for(int i=0;i<3;i++){
                V(i,2)*=-1;
            }
            R=svd.matrixU()*V.transpose();
        }
        Eigen::Vector3d t = center_dst - R*center_src;

        _R=R;
        _T=t;

    }
    Matrix3d& rot_matrix(){return _R;}
    Vector3d& translation(){return _T;}
    void set_rot_matrix(Matrix3d rot_mat){_R=rot_mat;}
    void set_translation(Vector3d trans){_T=trans;}

    PointsType load_point_cloud(const std::string &csv_file) {
        std::ifstream in;
        util::openfile_check(in, csv_file);
        csv::CsvReader reader(in);
        csv::CsvRow row;
        PointsType point_cloud;

        while (reader >> row) {
            point_cloud.emplace_back(Eigen::Vector3d(
                                         std::stod(row["x"]),
                                         std::stod(row["y"]),
                                         std::stod(row["z"])));
        }
        return point_cloud;
    }

    PointsType compute_transformed_points(PointsType p1s){
        PointsType p2s;
        for(auto& p:p1s){p2s.emplace_back(_R*p+_T);}
        return p2s;
    }

    Vector3d compute_transformed_points(QVector3D p1s){
        Vector3d p2s,p1;
        p1(0)=p1s.x();
        p1(1)=p1s.y();
        p1(2)=p1s.z();
        p2s=_R*p1+_T;
        return p2s;
    }

private:
    Matrix3d _R;
    Vector3d _T;

};

#endif
