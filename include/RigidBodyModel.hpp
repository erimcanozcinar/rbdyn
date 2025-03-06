#ifndef RIGIDBODYMODEL_HPP
#define RIGIDBODYMODEL_HPP

#include <tinyxml2.h>
#include <iostream>
#include <fstream>
#include "Dynamics.hpp"
#include "Kinematics.hpp"

class RigidBodyModel : public RigidBodyDynamics, public RigidBodyKinematics {    
    private:    
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
        _linkNames.push_back(std::string(link->Attribute("name")));
        _linkIDs.push_back(linkID);
        const char* xyz = link->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("xyz");
        const char* rpy = link->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("rpy");
        Eigen::Vector3d pCOM;
        Eigen::Vector3d rCOM;
        pCOM = string2Vec(xyz);
        rCOM = string2Vec(rpy);
        _linkCOMs.push_back(pCOM);
        _linkRots.push_back(rCOM);

        double mass;
        link->FirstChildElement("inertial")->FirstChildElement("mass")->QueryDoubleAttribute("value", &mass);
        _linkMasses.push_back(mass);

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
        _linkInertias.push_back(inertiaTensor);   
    }

    void parseJoint(const tinyxml2::XMLElement* joint) {
        jointID++;
        _jointNames.push_back(std::string(joint->Attribute("name")));
        _jointIDs.push_back(jointID+1);
        _jointTypes.push_back(jointTypeConvert(joint->Attribute("type")));

        _jointParents.push_back(std::string(joint->FirstChildElement("parent")->Attribute("link")));
        _jointChilds.push_back(std::string(joint->FirstChildElement("child")->Attribute("link")));
        _jointParentIDs.push_back(getBodyID(_jointParents[jointID]));        
        _jointChildIDs.push_back(getBodyID(_jointChilds[jointID]));
        
        _jointAxes.push_back(jointAxisConvert(joint->FirstChildElement("axis")->Attribute("xyz")));
        
        Eigen::Vector3d pJoint;
        Eigen::Vector3d rJoint;
        const char* jPos = joint->FirstChildElement("origin")->Attribute("xyz");
        const char* jRot = joint->FirstChildElement("origin")->Attribute("rpy");

        pJoint = string2Vec(jPos);
        rJoint = string2Vec(jRot);
        _jointLocations.push_back(pJoint);
        _jointRotations.push_back(rJoint);

    }
    
    int getDOF() {
        int dof = 0;
        for(JointType type : _jointTypes) {
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
            _jointNames.insert(_jointNames.begin(), "Floating Base");
            _jointIDs.insert(_jointIDs.begin(), 0);
            _jointTypes.insert(_jointTypes.begin(), JointType::Floating);
            _jointParents.insert(_jointParents.begin(), "world");
            _jointChilds.insert(_jointChilds.begin(), _linkNames[0]);
            _jointParentIDs.insert(_jointParentIDs.begin(), -1);        
            _jointChildIDs.insert(_jointChildIDs.begin(), getBodyID(_linkNames[0]));
            _jointAxes.insert(_jointAxes.begin(), CoordinateAxis::None);
            _jointLocations.insert(_jointLocations.begin(), Eigen::Vector3d::Zero());
            _jointRotations.insert(_jointRotations.begin(), Eigen::Vector3d::Zero());
        } else {
            _jointNames.insert(_jointNames.begin(), "Fixed Base");
            _jointIDs.insert(_jointIDs.begin(), 0);
            _jointTypes.insert(_jointTypes.begin(), JointType::Fixed);
            _jointParents.insert(_jointParents.begin(), "world");
            _jointChilds.insert(_jointChilds.begin(), std::string(baseChild));
            _jointParentIDs.insert(_jointParentIDs.begin(), -1);        
            _jointChildIDs.insert(_jointChildIDs.begin(), getBodyID(std::string(baseChild)));
            _jointAxes.insert(_jointAxes.begin(), CoordinateAxis::None);

            const char* baseJointPos = robot->FirstChildElement("joint")->FirstChildElement("origin")->Attribute("xyz");
            const char* baseJointRot = robot->FirstChildElement("joint")->FirstChildElement("origin")->Attribute("rpy");
            Eigen::Vector3d pJointBase;
            Eigen::Vector3d rJointBase;
            pJointBase = string2Vec(baseJointPos);
            rJointBase = string2Vec(baseJointRot);
            _jointLocations.insert(_jointLocations.begin(), pJointBase);
            _jointRotations.insert(_jointRotations.begin(), rJointBase);
        }

    }
  
    void printURDFInfo() {
        std::cout << "\033[35m  URDF INFORMATION \033[0m" << std::endl;

        std::cout << "\n";
        std::cout << "\033[32mLink Names: \033[0m" << std::endl;
        for(std::string name : _linkNames) {
            std::cout << name << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink IDs: \033[0m" << std::endl;
        for(int id : _linkIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink CoM: \033[0m" << std::endl;
        for(Eigen::Vector3d com : _linkCOMs) {
            std::cout << com.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Rot: \033[0m" << std::endl;
        for(Eigen::Vector3d rot : _linkRots) {
            std::cout << rot.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Mass: \033[0m" << std::endl;
        for(double mass : _linkMasses) {
            std::cout << mass << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Inertia: \033[0m" << std::endl;
        for(Eigen::Matrix3d inertia : _linkInertias) {
            std::cout << inertia << std::endl;
        }


        std::cout << "\n";
        std::cout << "\033[32mJoint Names: \033[0m" << std::endl;
        for(std::string name : _jointNames) {
            std::cout << name << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint IDs: \033[0m" << std::endl;
        for(int id : _jointIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Types: \033[0m" << std::endl;
        for(JointType type : _jointTypes) {
            std::cout << toString(type) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint _parents: \033[0m" << std::endl;
        for(std::string parent : _jointParents) {
            std::cout << parent << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Parent IDs: \033[0m" << std::endl;
        for(int id : _jointParentIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Childs: \033[0m" << std::endl;
        for(std::string child : _jointChilds) {
            std::cout << child << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Child IDs: \033[0m" << std::endl;
        for(int id : _jointChildIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Axes: \033[0m" << std::endl;
        for(CoordinateAxis axis : _jointAxes) {
            std::cout << toString(axis) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Locations: \033[0m" << std::endl;
        for(Eigen::Vector3d loc : _jointLocations) {
            std::cout << loc.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Rotations: \033[0m" << std::endl;
        for(Eigen::Vector3d rot : _jointRotations) {
            std::cout << rot.transpose() << std::endl;
        }
    }

    void printModelInfo() {
        std::cout << "\033[35m  RIGID BODY MODEL INFORMATION \033[0m" << std::endl;

        std::cout << "\n";
        std::cout << "\033[32m_Xtree: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << _Xtree[i] << std::endl;
            std::cout << "---------------\033[0m" << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Types: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << toString(_jointTypes[i]) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Axes: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << toString(_jointAxes[i]) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mParent Bodies: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << _parents[i] << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mSpatial Inertia: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << _Ibody[i].getMatrix() << std::endl;
            std::cout << "---------------" << std::endl;
        }
    }

    void createModel(const std::string& urdf_file_path) {
        read(urdf_file_path);
        
        nDof = getDOF();
        nBody = _linkNames.size(); 

        setGravity(Eigen::Vector3d(0,0,-9.81));

        for(int i=0; i<nBody; i++) {
            RotMat R = coordinateRotation(CoordinateAxis::Z, _jointRotations[i](2))*coordinateRotation(CoordinateAxis::Y, _jointRotations[i](1))*coordinateRotation(CoordinateAxis::X, _jointRotations[i](0));
            _Xtree.push_back(spatialTransform(R, _jointLocations[i]));
            SpatialInertia bodyInertia(_linkMasses[i], _linkCOMs[i], _linkInertias[i]);
            _parents.push_back(_jointParentIDs[i]);
            _Ibody.push_back(bodyInertia);

            _gravity.template tail<3>() = gravity;
        }
        
        printURDFInfo();
        printModelInfo();

    }

    public:

    Eigen::Vector3d gravity;

    RigidBodyModel(const std::string& urdf_file_path) {
        createModel(urdf_file_path);
        initDynamics();
        initKinematics();
    }

    int getBodyID(const std::string& name) {
        auto it = std::find(_linkNames.begin(), _linkNames.end(), name);
        
        if (it != _linkNames.end()) {
            return std::distance(_linkNames.begin(), it);  // Return the index of the found link
        } else {
            throw std::runtime_error("Link does not exist");
            return -1;
        }
    }

    int getJointID(const std::string& name) {
        auto it = std::find(_jointNames.begin(), _jointNames.end(), name);
        
        if (it != _jointNames.end()) {
            return std::distance(_jointNames.begin(), it);  // Return the index of the found link
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