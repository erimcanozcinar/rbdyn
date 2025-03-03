#ifndef RIGIDBODYMODEL_HPP
#define RIGIDBODYMODEL_HPP

#include <tinyxml2.h>
#include <iostream>
#include <fstream>
#include "SpatialInertia.hpp"
using namespace spatial;

class RigidBodyModel {    
    private:    
    bool isFixedBase = false;
    int linkID = -1;
    int jointID = -1;

    std::vector<std::string> linkNames;
    std::vector<int> linkIDs;
    std::vector<Eigen::Vector3d> linkCOMs;
    std::vector<Eigen::Vector3d> linkRots;
    std::vector<double> linkMasses;
    std::vector<Eigen::Matrix3d> linkInertias;

    std::vector<std::string> jointNames;
    std::vector<int> jointIDs;
    std::vector<std::string> jointParents;
    std::vector<std::string> jointChilds;    
    std::vector<Eigen::Vector3d> jointRotations;

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
        linkNames.push_back(std::string(link->Attribute("name")));
        linkIDs.push_back(linkID);
        const char* xyz = link->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("xyz");
        const char* rpy = link->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("rpy");
        Eigen::Vector3d pCOM;
        Eigen::Vector3d rCOM;
        pCOM = string2Vec(xyz);
        rCOM = string2Vec(rpy);
        linkCOMs.push_back(pCOM);
        linkRots.push_back(rCOM);

        double mass;
        link->FirstChildElement("inertial")->FirstChildElement("mass")->QueryDoubleAttribute("value", &mass);
        linkMasses.push_back(mass);

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
        linkInertias.push_back(inertiaTensor);   
    }

    void parseJoint(const tinyxml2::XMLElement* joint) {
        jointID++;
        jointNames.push_back(std::string(joint->Attribute("name")));
        jointIDs.push_back(jointID+1);
        jointTypes.push_back(jointTypeConvert(joint->Attribute("type")));

        jointParents.push_back(std::string(joint->FirstChildElement("parent")->Attribute("link")));
        jointChilds.push_back(std::string(joint->FirstChildElement("child")->Attribute("link")));
        jointParentIDs.push_back(getLinkID(jointParents[jointID]));        
        jointChildIDs.push_back(getLinkID(jointChilds[jointID]));
        
        jointAxes.push_back(jointAxisConvert(joint->FirstChildElement("axis")->Attribute("xyz")));
        
        Eigen::Vector3d pJoint;
        Eigen::Vector3d rJoint;
        const char* jPos = joint->FirstChildElement("origin")->Attribute("xyz");
        const char* jRot = joint->FirstChildElement("origin")->Attribute("rpy");

        pJoint = string2Vec(jPos);
        rJoint = string2Vec(jRot);
        jointLocations.push_back(pJoint);
        jointRotations.push_back(rJoint);

    }
    
    int getDOF() {
        int dof = 0;
        for(JointType type : jointTypes) {
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
            jointNames.insert(jointNames.begin(), "Floating Base");
            jointIDs.insert(jointIDs.begin(), 0);
            jointTypes.insert(jointTypes.begin(), JointType::Floating);
            jointParents.insert(jointParents.begin(), "world");
            jointChilds.insert(jointChilds.begin(), linkNames[0]);
            jointParentIDs.insert(jointParentIDs.begin(), -1);        
            jointChildIDs.insert(jointChildIDs.begin(), getLinkID(linkNames[0]));
            jointAxes.insert(jointAxes.begin(), CoordinateAxis::None);
            jointLocations.insert(jointLocations.begin(), Eigen::Vector3d::Zero());
            jointRotations.insert(jointRotations.begin(), Eigen::Vector3d::Zero());
        } else {
            jointNames.insert(jointNames.begin(), "Fixed Base");
            jointIDs.insert(jointIDs.begin(), 0);
            jointTypes.insert(jointTypes.begin(), JointType::Fixed);
            jointParents.insert(jointParents.begin(), "world");
            jointChilds.insert(jointChilds.begin(), std::string(baseChild));
            jointParentIDs.insert(jointParentIDs.begin(), -1);        
            jointChildIDs.insert(jointChildIDs.begin(), getLinkID(std::string(baseChild)));
            jointAxes.insert(jointAxes.begin(), CoordinateAxis::None);

            const char* baseJointPos = robot->FirstChildElement("joint")->FirstChildElement("origin")->Attribute("xyz");
            const char* baseJointRot = robot->FirstChildElement("joint")->FirstChildElement("origin")->Attribute("rpy");
            Eigen::Vector3d pJointBase;
            Eigen::Vector3d rJointBase;
            pJointBase = string2Vec(baseJointPos);
            rJointBase = string2Vec(baseJointRot);
            jointLocations.insert(jointLocations.begin(), pJointBase);
            jointRotations.insert(jointRotations.begin(), rJointBase);
        }

    }
  
    void printURDFInfo() {
        std::cout << "\033[35m  URDF INFORMATION \033[0m" << std::endl;

        std::cout << "\n";
        std::cout << "\033[32mLink Names: \033[0m" << std::endl;
        for(std::string name : linkNames) {
            std::cout << name << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink IDs: \033[0m" << std::endl;
        for(int id : linkIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink CoM: \033[0m" << std::endl;
        for(Eigen::Vector3d com : linkCOMs) {
            std::cout << com.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Rot: \033[0m" << std::endl;
        for(Eigen::Vector3d rot : linkRots) {
            std::cout << rot.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Mass: \033[0m" << std::endl;
        for(double mass : linkMasses) {
            std::cout << mass << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mLink Inertia: \033[0m" << std::endl;
        for(Eigen::Matrix3d inertia : linkInertias) {
            std::cout << inertia << std::endl;
        }


        std::cout << "\n";
        std::cout << "\033[32mJoint Names: \033[0m" << std::endl;
        for(std::string name : jointNames) {
            std::cout << name << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint IDs: \033[0m" << std::endl;
        for(int id : jointIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Types: \033[0m" << std::endl;
        for(JointType type : jointTypes) {
            std::cout << toString(type) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Parents: \033[0m" << std::endl;
        for(std::string parent : jointParents) {
            std::cout << parent << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Parent IDs: \033[0m" << std::endl;
        for(int id : jointParentIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Childs: \033[0m" << std::endl;
        for(std::string child : jointChilds) {
            std::cout << child << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Child IDs: \033[0m" << std::endl;
        for(int id : jointChildIDs) {
            std::cout << id << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Axes: \033[0m" << std::endl;
        for(CoordinateAxis axis : jointAxes) {
            std::cout << toString(axis) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Locations: \033[0m" << std::endl;
        for(Eigen::Vector3d loc : jointLocations) {
            std::cout << loc.transpose() << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Rotations: \033[0m" << std::endl;
        for(Eigen::Vector3d rot : jointRotations) {
            std::cout << rot.transpose() << std::endl;
        }
    }

    void printModelInfo() {
        std::cout << "\033[35m  RIGID BODY MODEL INFORMATION \033[0m" << std::endl;

        std::cout << "\n";
        std::cout << "\033[32mXtree: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << Xtree[i] << std::endl;
            std::cout << "---------------\033[0m" << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Types: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << toString(jointTypes[i]) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mJoint Axes: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << toString(jointAxes[i]) << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mParent Bodies: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << parents[i] << std::endl;
        }
        std::cout << "\n";
        std::cout << "\033[32mSpatial Inertia: \033[0m" << std::endl;
        for(int i=0; i<nBody; i++) {
            std::cout << Ibody[i].getMatrix() << std::endl;
            std::cout << "---------------" << std::endl;
        }
    }

    public:

    int nBody; // number of rigid bodies (including fixed base)
    int nDof; // number of moveable joints

    std::vector<int> jointParentIDs;
    std::vector<int> jointChildIDs;
    std::vector<Eigen::Vector3d> jointLocations;

    Eigen::Vector3d gravity; 
    std::vector<sMat, Eigen::aligned_allocator<sMat>> Xtree;
    std::vector<JointType> jointTypes;
    std::vector<CoordinateAxis> jointAxes;
    std::vector<int> parents;
    std::vector<SpatialInertia, Eigen::aligned_allocator<SpatialInertia>> Ibody;

    int getLinkID(const std::string& name) {
        auto it = std::find(linkNames.begin(), linkNames.end(), name);
        
        if (it != linkNames.end()) {
            return std::distance(linkNames.begin(), it);  // Return the index of the found link
        } else {
            throw std::runtime_error("Link does not exist");
            return -1;
        }
    }

    int getJointID(const std::string& name) {
        auto it = std::find(jointNames.begin(), jointNames.end(), name);
        
        if (it != jointNames.end()) {
            return std::distance(jointNames.begin(), it);  // Return the index of the found link
        } else {
            throw std::runtime_error("Link does not exist");
            return -1;
        }
    }

    void createModel(const std::string& urdf_file_path) {
        read(urdf_file_path);
        
        nDof = getDOF();
        nBody = linkNames.size();

        for(int i=0; i<nBody; i++) {
            RotMat R = coordinateRotation(CoordinateAxis::Z, jointRotations[i](2))*coordinateRotation(CoordinateAxis::Y, jointRotations[i](1))*coordinateRotation(CoordinateAxis::X, jointRotations[i](0));
            Xtree.push_back(spatialTransform(R, jointLocations[i]));
            SpatialInertia bodyInertia(linkMasses[i], linkCOMs[i], linkInertias[i]);
            parents.push_back(jointParentIDs[i]);
            Ibody.push_back(bodyInertia);
        }
        printURDFInfo();
        printModelInfo();

    }

};

#endif