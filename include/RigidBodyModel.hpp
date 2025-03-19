#ifndef RIGIDBODYMODEL_HPP
#define RIGIDBODYMODEL_HPP

#include <tinyxml2.h>
#include <iostream>
#include <fstream>
#include "Dynamics.hpp"
#include "Kinematics.hpp"

class RigidBodyModel : public RigidBodyDynamics, public RigidBodyKinematics {    
    private:    
    ModelParameters _urdf;
    bool isFixedBase = false;
    int linkID = -1;
    int jointID = -1;

    JointType jointTypeConvert(const char* joint_type) {
        if(strcmp(joint_type, "revolute") == 0)
            return JointType::Revolute;
        else if(strcmp(joint_type, "prismatic") == 0)
            return JointType::Prismatic;
        else if(strcmp(joint_type, "fixed") == 0)
            return JointType::Fixed;
        else
            return JointType::Floating;
    }

    CoordinateAxis jointAxisConvert(const char* joint_axis) {
        if(strcmp(joint_axis, "1 0 0") == 0 || strcmp(joint_axis, "-1 0 0") == 0)
            return CoordinateAxis::X;
        else if(strcmp(joint_axis, "0 1 0") == 0 || strcmp(joint_axis, "0 -1 0") == 0)
            return CoordinateAxis::Y;
        else if(strcmp(joint_axis, "0 0 1") == 0 || strcmp(joint_axis, "0 0 -1") == 0)
            return CoordinateAxis::Z;
        else
            return CoordinateAxis::None;

    }

    std::string loadURDFContent(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open URDF file: " + file_path);
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    Eigen::Vector3d string2Vec(const char* _Str) {
    Eigen::Vector3d vec(0.0, 0.0, 0.0);
    if (_Str) {
        std::stringstream ss(_Str);
        ss >> vec.x() >> vec.y() >> vec.z();
    }
    return vec;
}
    
    void parseLink(const tinyxml2::XMLElement* link) {
        linkID++;
        _urdf._linkNames.push_back(std::string(link->Attribute("name")));
        _urdf._linkIDs.push_back(linkID);
        const char* xyz = link->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("xyz");
        const char* rpy = link->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("rpy");
        Eigen::Vector3d pCOM;
        Eigen::Vector3d rCOM;
        pCOM = string2Vec(xyz);
        rCOM = string2Vec(rpy);
        _urdf._linkCOMs.push_back(pCOM);
        _urdf._linkRots.push_back(rCOM);

        double mass;
        link->FirstChildElement("inertial")->FirstChildElement("mass")->QueryDoubleAttribute("value", &mass);
        _urdf._linkMasses.push_back(mass);

        double Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
        Eigen::Matrix3d inertiaTensor;
        link->FirstChildElement("inertial")->FirstChildElement("inertia")->QueryDoubleAttribute("ixx", &Ixx);
        link->FirstChildElement("inertial")->FirstChildElement("inertia")->QueryDoubleAttribute("ixy", &Ixy);
        link->FirstChildElement("inertial")->FirstChildElement("inertia")->QueryDoubleAttribute("ixz", &Ixz);
        link->FirstChildElement("inertial")->FirstChildElement("inertia")->QueryDoubleAttribute("iyy", &Iyy);
        link->FirstChildElement("inertial")->FirstChildElement("inertia")->QueryDoubleAttribute("iyz", &Iyz);
        link->FirstChildElement("inertial")->FirstChildElement("inertia")->QueryDoubleAttribute("izz", &Izz);
        inertiaTensor << Ixx, Ixy, Ixz,
                   Ixy, Iyy, Iyz,
                   Ixz, Iyz, Izz;
        _urdf._linkInertias.push_back(inertiaTensor);   
    }

    void parseJoint(const tinyxml2::XMLElement* joint) {
        jointID++;
        _urdf._jointNames.push_back(std::string(joint->Attribute("name")));
        _urdf._jointIDs.push_back(jointID+1);
        _urdf._jointTypes.push_back(jointTypeConvert(joint->Attribute("type")));

        _urdf._jointParents.push_back(std::string(joint->FirstChildElement("parent")->Attribute("link")));
        _urdf._jointChilds.push_back(std::string(joint->FirstChildElement("child")->Attribute("link")));
        _urdf._jointParentIDs.push_back(getBodyID(_urdf._jointParents[jointID]));        
        _urdf._jointChildIDs.push_back(getBodyID(_urdf._jointChilds[jointID]));
        
        if(joint->FirstChildElement("axis")) {
            _urdf._jointAxes.push_back(jointAxisConvert(joint->FirstChildElement("axis")->Attribute("xyz")));
        } else {
            _urdf._jointAxes.push_back(CoordinateAxis::None);
        }
        
        Eigen::Vector3d pJoint;
        Eigen::Vector3d rJoint;
        const char* jPos = joint->FirstChildElement("origin")->Attribute("xyz");
        const char* jRot = joint->FirstChildElement("origin")->Attribute("rpy");

        pJoint = string2Vec(jPos);
        rJoint = string2Vec(jRot);
        _urdf._jointLocations.push_back(pJoint);
        _urdf._jointRotations.push_back(rJoint);

    }
    
    int getDOF() {
        int dof = 0;
        for(JointType type : _urdf._jointTypes) {
            if (type == JointType::Floating)
                dof = dof + 6;
            else if (type == JointType::Fixed)
                dof = dof + 0;
            else if (type == JointType::Spherical)
                dof = dof + 3;
            else
                dof = dof + 1;
        }
        return dof;
    }

    void read(const std::string& urdf_file_path) {
        std::string urdf_content;

        try {
            urdf_content = loadURDFContent(urdf_file_path);
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }

        tinyxml2::XMLDocument doc;
        if (doc.Parse(urdf_content.c_str()) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Failed to parse URDF file: " << doc.ErrorStr() << std::endl;
        }

        const tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
        if (!robot) {
            std::cerr << "No <robot> element found in URDF file." << std::endl;
        }

        const tinyxml2::XMLElement* firstLink = robot->FirstChildElement("link");
        const tinyxml2::XMLElement* firstJoint = robot->FirstChildElement("joint");
        if(strcmp(firstLink->Attribute("name"), "world") == 0 || strcmp(firstLink->Attribute("name"), "World") == 0) {
            firstLink = robot->FirstChildElement("link")->NextSiblingElement("link");
            firstJoint = robot->FirstChildElement("joint")->NextSiblingElement("joint");
            isFixedBase = true;
        }

        for (const tinyxml2::XMLElement* link = firstLink; link; link = link->NextSiblingElement("link")) {
            parseLink(link);
        }

        for (const tinyxml2::XMLElement* joint = firstJoint; joint; joint = joint->NextSiblingElement("joint")) {
            parseJoint(joint);
        }

        
        const char* baseChild = robot->FirstChildElement("joint")->FirstChildElement("child")->Attribute("link");
        if(!isFixedBase) {
            _urdf._jointNames.insert(_urdf._jointNames.begin(), "Floating Base");
            _urdf._jointIDs.insert(_urdf._jointIDs.begin(), 0);
            _urdf._jointTypes.insert(_urdf._jointTypes.begin(), JointType::Floating);
            _urdf._jointParents.insert(_urdf._jointParents.begin(), "world");
            _urdf._jointChilds.insert(_urdf._jointChilds.begin(), _urdf._linkNames[0]);
            _urdf._jointParentIDs.insert(_urdf._jointParentIDs.begin(), -1);        
            _urdf._jointChildIDs.insert(_urdf._jointChildIDs.begin(), getBodyID(_urdf._linkNames[0]));
            _urdf._jointAxes.insert(_urdf._jointAxes.begin(), CoordinateAxis::None);
            _urdf._jointLocations.insert(_urdf._jointLocations.begin(), Eigen::Vector3d::Zero());
            _urdf._jointRotations.insert(_urdf._jointRotations.begin(), Eigen::Vector3d::Zero());
        } else {
            _urdf._jointNames.insert(_urdf._jointNames.begin(), "Fixed Base");
            _urdf._jointIDs.insert(_urdf._jointIDs.begin(), 0);
            _urdf._jointTypes.insert(_urdf._jointTypes.begin(), JointType::Fixed);
            _urdf._jointParents.insert(_urdf._jointParents.begin(), "world");
            _urdf._jointChilds.insert(_urdf._jointChilds.begin(), std::string(baseChild));
            _urdf._jointParentIDs.insert(_urdf._jointParentIDs.begin(), -1);        
            _urdf._jointChildIDs.insert(_urdf._jointChildIDs.begin(), getBodyID(std::string(baseChild)));
            _urdf._jointAxes.insert(_urdf._jointAxes.begin(), CoordinateAxis::None);

            const char* baseJointPos = robot->FirstChildElement("joint")->FirstChildElement("origin")->Attribute("xyz");
            const char* baseJointRot = robot->FirstChildElement("joint")->FirstChildElement("origin")->Attribute("rpy");
            Eigen::Vector3d pJointBase;
            Eigen::Vector3d rJointBase;
            pJointBase = string2Vec(baseJointPos);
            rJointBase = string2Vec(baseJointRot);
            _urdf._jointLocations.insert(_urdf._jointLocations.begin(), pJointBase);
            _urdf._jointRotations.insert(_urdf._jointRotations.begin(), rJointBase);
        }

    }
  
    void printURDFInfo() {
        std::cout << "\033[35m  URDF INFORMATION \033[0m" << std::endl;

        std::cout << "\n";
        std::cout << "\033[32mLink Names: \033[0m" << std::endl;
        for(std::string name : _urdf._linkNames) {
            std::cout << name << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink IDs: \033[0m" << std::endl;
        for(int id : _urdf._linkIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink CoM: \033[0m" << std::endl;
        for(Eigen::Vector3d com : _urdf._linkCOMs) {
            std::cout << com.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Rot: \033[0m" << std::endl;
        for(Eigen::Vector3d rot : _urdf._linkRots) {
            std::cout << rot.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Mass: \033[0m" << std::endl;
        for(double mass : _urdf._linkMasses) {
            std::cout << mass << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Inertia: \033[0m" << std::endl;
        for(Eigen::Matrix3d inertia : _urdf._linkInertias) {
            std::cout << inertia << std::endl;
        }


        std::cout << "\n";
        std::cout << "\033[32mJoint Names: \033[0m" << std::endl;
        for(std::string name : _urdf._jointNames) {
            std::cout << name << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint IDs: \033[0m" << std::endl;
        for(int id : _urdf._jointIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Types: \033[0m" << std::endl;
        for(JointType type : _urdf._jointTypes) {
            std::cout << toString(type) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint _urdf._parents: \033[0m" << std::endl;
        for(std::string parent : _urdf._jointParents) {
            std::cout << parent << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Parent IDs: \033[0m" << std::endl;
        for(int id : _urdf._jointParentIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Childs: \033[0m" << std::endl;
        for(std::string child : _urdf._jointChilds) {
            std::cout << child << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Child IDs: \033[0m" << std::endl;
        for(int id : _urdf._jointChildIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Axes: \033[0m" << std::endl;
        for(CoordinateAxis axis : _urdf._jointAxes) {
            std::cout << toString(axis) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Locations: \033[0m" << std::endl;
        for(Eigen::Vector3d loc : _urdf._jointLocations) {
            std::cout << loc.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Rotations: \033[0m" << std::endl;
        for(Eigen::Vector3d rot : _urdf._jointRotations) {
            std::cout << rot.transpose() << std::endl;
        }
    }

    void printModelInfo() {
        std::cout << "\033[35m  RIGID BODY MODEL INFORMATION \033[0m" << std::endl;

        std::cout << "\n";
        std::cout << "\033[32m_urdf._Xtree: \033[0m" << std::endl;
        for(int i=0; i<_urdf.nBody; i++) {
            std::cout << _urdf._Xtree[i] << std::endl;
            std::cout << "---------------\033[0m" << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Types: \033[0m" << std::endl;
        for(int i=0; i<_urdf.nBody; i++) {
            std::cout << toString(_urdf._jointTypes[i]) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Axes: \033[0m" << std::endl;
        for(int i=0; i<_urdf.nBody; i++) {
            std::cout << toString(_urdf._jointAxes[i]) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mParent Bodies: \033[0m" << std::endl;
        for(int i=0; i<_urdf.nBody; i++) {
            std::cout << _urdf._parents[i] << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mSpatial Inertia: \033[0m" << std::endl;
        for(int i=0; i<_urdf.nBody; i++) {
            std::cout << _urdf._Ibody[i].getMatrix() << std::endl;
            std::cout << "---------------" << std::endl;
        }
    }

    void createModel(const std::string& urdf_file_path) {
        read(urdf_file_path);
        
        _urdf.nDof = getDOF();
        _urdf.nBody = _urdf._linkNames.size(); 

        setGravity(Eigen::Vector3d(0,0,-9.81));

        _urdf._Sf.resize(_urdf.nDof, _urdf.nBody);
        _urdf._Sf.setZero();
        Eigen::RowVectorXd sF(_urdf.nBody);

        for(int i=0; i<_urdf.nBody; i++) {
            RotMat R = coordinateRotation(CoordinateAxis::Z, _urdf._jointRotations[i](2))*coordinateRotation(CoordinateAxis::Y, _urdf._jointRotations[i](1))*coordinateRotation(CoordinateAxis::X, _urdf._jointRotations[i](0));
            _urdf._Xtree.push_back(spatialTransform(R, _urdf._jointLocations[i]));
            SpatialInertia bodyInertia(_urdf._linkMasses[i], _urdf._linkCOMs[i], _urdf._linkInertias[i]);
            _urdf._parents.push_back(_urdf._jointParentIDs[i]);
            _urdf._Ibody.push_back(bodyInertia);
            _urdf._q.push_back(0.0);
            _urdf._dq.push_back(0.0);
            _urdf._ddq.push_back(0.0);

            sF.setZero();
            if(_urdf._jointTypes[i] != JointType::Fixed) {
                sF(_urdf._jointIDs[i]) = 1;
            }    
            std::cout<< sF << std::endl;

            // for(int j=0; j<_urdf.nDof; j++){
            //     _urdf._Sf.row(i) = sF;
            // }     

            _urdf._gravity.template tail<3>() = gravity;
        }
        // printURDFInfo();
        // printModelInfo();

    }

    public:

    Eigen::Vector3d gravity;

    RigidBodyModel(const std::string& urdf_file_path) {
        createModel(urdf_file_path);
        initDynamics(_urdf);
        // initKinematics(_urdf);
    }

    int getBodyID(const std::string& name) {
        auto it = std::find(_urdf._linkNames.begin(), _urdf._linkNames.end(), name);
        
        if (it != _urdf._linkNames.end()) {
            return std::distance(_urdf._linkNames.begin(), it);  // Return the index of the found link
        } else {
            throw std::runtime_error("Link does not exist");
            return -1;
        }
    }

    int getJointID(const std::string& name) {
        auto it = std::find(_urdf._jointNames.begin(), _urdf._jointNames.end(), name);
        
        if (it != _urdf._jointNames.end()) {
            return std::distance(_urdf._jointNames.begin(), it);  // Return the index of the found link
        } else {
            throw std::runtime_error("Link does not exist");
            return -1;
        }
    }

    void setGravity(Eigen::Vector3d grav) {
        gravity = grav;
    }

    

};

#endif