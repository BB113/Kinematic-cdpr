#!/usr/bin/python3

from mod_create_2 import *
import yaml
import sys
import numpy as np
from math import *
import transformations as tr
from os.path import exists

#j'ai récupéré ce code pour créer un fichier urdf et non sdf
# les liaisons sphériques au exremités du cable sont modélisées par une mise en série de liaison pivot#

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print(' Give a yaml file' )
        sys.exit(0)

    robot = sys.argv[1]
    if not exists(robot):
        for ext in ['.yaml', '.yml', 'yaml','yml']:
            if exists(robot + ext):
                robot += ext
                break
    if not exists(robot):
        print(robot + ' not found')
        sys.exit(0)

    #d_config = yaml.load(file(robot))

    yaml_file = open("cube1.yaml", "r+")
    d_config = yaml.load(yaml_file, Loader=yaml.SafeLoader)

    sim_cables = True
    if 'sim_cables' in d_config:
        sim_cables = d_config['sim_cables']

    # check point values are all doubles for C++ parser
    for i in range(len(d_config['points'])):
        for j in range(3):
            if sim_cables:
                d_config['points'][i]['frame'][j] = float(d_config['points'][i]['frame'][j])
            d_config['points'][i]['platform'][j] = float(d_config['points'][i]['platform'][j])
    # same check for inertia matrix
    for i in range(6):
        d_config['platform']['inertia'][i] = float(d_config['platform']['inertia'][i])

    # re-write config
    with open(robot,'w') as f:
            yaml.dump(d_config, f)

    config = DictsToNamespace(d_config)
    config.frame.upper = [float(v) for v in config.frame.upper]
    config.frame.lower = [float(v) for v in config.frame.lower]
    name = robot.split('.')[0]

    robot = etree.Element('robot')
    """
    # frame
    robot.insert(2, etree.Comment('Definition of the robot frame'))
    gazebo=etree.SubElement(robot, 'gazebo')

    base_link = etree.SubElement(gazebo, 'link', name= 'frame')
    CreateNested(base_link, 'pose', '0 0 0 0 0 0')
    BuildInertial(base_link, 100000)
    """

    # frame visual
    if config.frame.type == 'box':
        # default visual: cubic frame
        # find corner points
        points = []
        lx,ly,lz = [config.frame.upper[i] - config.frame.lower[i] for i in range(3)]
        for dx in [0,1]:
            for dy in [0,1]:
                for dz in [0,1]:
                    dxyz = [dx*lx, dy*ly, dz*lz]
                    points.append([config.frame.lower[i]+dxyz[i] for i in range(3)])

        # create segments
        ident = 0
        for i,p1 in enumerate(points[:-1]):
            for p2 in points[i+1:]:
                dp = [p2[i]-p1[i] for i in range(3)]
                if dp.count(0) == 2:
                    # middle of segment
                    pose = [p1[i]+dp[i]/2. for i in range(3)] + [0,0,0]
                    # find orientation
                    if dp[0] != 0:
                        pose[4] = pi/2
                    elif dp[1] != 0:
                        pose[3] = pi/2
                    # create link
                    ident += 1

    #create platform data
    if config.platform.type == 'box':
        pose = config.platform.position.xyz + config.platform.position.rpy
    # platform translation and rotation
    pf_t = np.array(config.platform.position.xyz).reshape(3,1)
    pf_R = tr.euler_matrix(config.platform.position.rpy[0], config.platform.position.rpy[1], config.platform.position.rpy[2])[:3,:3]
    # maximum length
    l = np.linalg.norm([config.frame.upper[i] - config.frame.lower[i] for i in range(3)])



    # create cables
    if sim_cables:
        robot.insert(2, etree.Comment('Definition of the robot cables'))
        z = [0,0,1]
        for i, cbl in enumerate(config.points):

            fp = np.array(cbl.frame).reshape(3,1)  # frame attach point
            # express platform attach point in world frame
            pp = pf_t + np.dot(pf_R, np.array(cbl.platform).reshape(3,1))
            # cable orientation
            u = (pp - fp).reshape(3)
            u = list(u/np.linalg.norm(u))
            R = tr.rotation_matrix(np.arctan2(np.linalg.norm(np.cross(z,u)), np.dot(u,z)), np.cross(z,u))
            # to RPY
            rpy_gen = list(tr.euler_from_matrix(R))
            # rpy of z-axis
            # cable position to stick to the platform
            a = l/(2.*np.linalg.norm(pp-fp))
            cp = list((pp - a*(pp-fp)).reshape(3))
            # create cable
            link = etree.SubElement(robot, 'link', name= 'cable%i' % i)

            collision = etree.SubElement(link, 'collision')
            l_xyz = ('%f %f %f %f %f %f' % tuple(cp + rpy_gen)).split(" ")
            l_xyz_2=('%f %f %f %f %f %f' % tuple(cbl.frame + rpy_gen)).split(" ")
            #xyz_c = str(l_xyz[0]+" "+l_xyz[1]+" "+l_xyz[2])
            xyz_c =str(l_xyz[0]+" "+l_xyz[1]+" "+l_xyz[2])
            offset = np.linalg.norm(np.array(cbl.frame)-pp.reshape(3))-l/2
            print(offset)
            rpy= str(l_xyz[3]+" "+l_xyz[4]+" "+l_xyz[5])

            origin=etree.SubElement(collision, 'origin', xyz ='0 0 '+str(offset), rpy = "0 0 0")
            geometry=etree.SubElement(collision, 'geometry')
            cylinder=etree.SubElement(geometry, 'cylinder',radius = str(config.cable.radius), length = str(l))

            visual = etree.SubElement(link, 'visual')

            #origin=etree.SubElement(visual, 'origin', xyz = xyz_c, rpy = rpy)
            origin=etree.SubElement(visual, 'origin', xyz ='0 0 '+str(offset), rpy = "0 0 0")
            geometry=etree.SubElement(visual, 'geometry')
            cylinder=etree.SubElement(geometry, 'cylinder',radius = str(config.cable.radius), length = str(l))
            material=etree.SubElement(visual, 'material', name = "${cable_color}")

            inertial = etree.SubElement(link, 'inertial')

            origin=etree.SubElement(inertial, 'origin', xyz ='0 0 '+str(offset), rpy = "0 0 0")
            geometry=etree.SubElement(inertial, 'geometry')
            cylinder=etree.SubElement(geometry, 'cylinder',radius = str(config.cable.radius), length = str(l))
            inertia=etree.SubElement(inertial, 'inertia', ixx="0.001", ixy="0", ixz="0", iyy="0.001", iyz="0", izz="0.001")
            mass=etree.SubElement(inertial, 'mass', value = "0.001")

# virtual link around X
            link = etree.SubElement(robot, 'link', name= 'virt_X%i' % i)
            inertial = etree.SubElement(link, 'inertial')
            l_2 = ('%f %f %f %f %f %f' % tuple(cbl.frame + rpy_gen)).split(" ")
            xyz = str(l_2[0]+" "+l_2[1]+" "+l_2[2])
            rpy = str(l_2[3]+" "+l_2[4]+" "+l_2[5])
            mass=etree.SubElement(inertial, 'mass', value = "0.001")
            inertia=etree.SubElement(inertial, 'inertia', ixx="0.001", ixy="0", ixz="0", iyy="0.001", iyz="0", izz="0.001")
            #CreateVisualCollision(link,'/geometry/cylinder/radius', .03, color='Red', collision=False)
            #CreateNested(link, 'visual/geometry/cylinder/length', 0.3)
            visual = etree.SubElement(link, 'visual')
            origin=etree.SubElement(inertial, 'origin', xyz = "0 0 0", rpy = "0 0 0")
            geometry=etree.SubElement(visual, 'geometry')
            cylinder=etree.SubElement(geometry, 'box',size = "0.1 0.1 0.1")
            material=etree.SubElement(visual, 'material', name = "${cable_color}")
            origin=etree.SubElement(visual, 'origin', xyz = "0 0 0", rpy = "0 0 0")

            # revolute joint around X
            joint = etree.SubElement(robot, 'joint', name= 'rev_X%i' % i)
            joint.set("type", "revolute")
            l_3 = ('0 0 0 %f %f %f' % tuple(rpy_gen)).split(" ")
            xyz_rjx = str(l_3[0]+" "+l_3[1]+" "+l_3[2])
            rot_rjx = str(l_3[3]+" "+l_3[4]+" "+l_3[5])
            origin=etree.SubElement(joint, 'origin', xyz = xyz, rpy = rot_rjx)
            parent_link=etree.SubElement(joint, 'parent', link = "frame_1")
            child_link=etree.SubElement(joint, 'child', link = 'virt_X%i' % i)
            axe=etree.SubElement(joint, 'axis', xyz = '%f %f %f' % tuple(R[:3,0]))
            limit=etree.SubElement(joint, 'limit', effort=str(config.joints.passive.effort), velocity=str(config.joints.passive.velocity))
            dynamics=etree.SubElement(joint, 'limit', damping=str(config.joints.passive.damping))

            # virtual link around Y
            link = etree.SubElement(robot, 'link', name= 'virt_Y%i' % i) 

            inertial = etree.SubElement(link, 'inertial')
            l_4 = ('%f %f %f %f %f %f' % tuple(cbl.frame + rpy_gen)).split(" ")
            xyz_vlx = str(l_4[0]+" "+l_4[1]+" "+l_4[2])
            rot_vlx = str(l_4[3]+" "+l_4[4]+" "+l_4[5])

            origin=etree.SubElement(inertial, 'origin', xyz = "0 0 0", rpy = "0 0 0")
            mass=etree.SubElement(inertial, 'mass', value = "0.001")
            inertia=etree.SubElement(inertial, 'inertia', ixx="0.001", ixy="0", ixz="0", iyy="0.001", iyz="0", izz="0.001")
            visual = etree.SubElement(link, 'visual')
            origin=etree.SubElement(visual, 'origin', xyz = "0 0 0", rpy = "0 0 0")
            geometry=etree.SubElement(visual, 'geometry')
            cylinder=etree.SubElement(geometry, 'box',size = "0.1 0.1 0.1")
            material=etree.SubElement(visual, 'material', name = "${cable_color}")


            # revolute joint around Y
            joint = etree.SubElement(robot, 'joint', name= 'rev_Y%i' % i)
            joint.set("type", "revolute")   
            l_5 = ('0 0 0 %f %f %f' % tuple(rpy_gen)).split(" ")
            xyz_rjx = str(l_5[0]+" "+l_5[1]+" "+l_5[2])
            rot_rjx = str(l_5[3]+" "+l_5[4]+" "+l_5[5])
            origin=etree.SubElement(joint, 'origin', xyz = "0 0 0", rpy = "0 0 0")
            parent_link=etree.SubElement(joint, 'parent', link = 'virt_X%i' % i)
            child_link=etree.SubElement(joint, 'child', link = 'virt_Y%i' % i)
            axe=etree.SubElement(joint, 'axis', xyz = '%f %f %f' % tuple(R[:3,1]))
            limit=etree.SubElement(joint, 'limit', effort=str(config.joints.passive.effort), velocity=str(config.joints.passive.velocity))
            dynamics=etree.SubElement(joint, 'limit', damping=str(config.joints.passive.damping))   



            # prismatic joint
            joint = etree.SubElement(robot, 'joint', name= 'joint_cable%i' % i)
            joint.set("type", "prismatic")
            #l_6 = ('0 0 %f %f %f %f' % tuple([(a-1.)*l/2] + rpy_gen)).split(" ")
            l_6 = ('0 0 0 %f %f %f' % tuple(rpy_gen)).split(" ")
            xyz_pjx = str(l_6[0]+" "+l_6[1]+" "+l_6[2])
            rot_pjx = str(l_6[3]+" "+l_6[4]+" "+l_6[5])
            print("prismatic" + str(str(float(l_2[0])-float(l_xyz[0]))+" "+str(float(l_2[1])-float(l_xyz[1]))+" "+str(float(l_2[2])-float(l_xyz[2]))))
            #origin=etree.SubElement(joint, 'origin', xyz = str(str(float(l_2[0])-float(l_xyz[0]))+" "+str(float(l_2[1])-float(l_xyz[1]))+" "+str(float(l_2[2])-float(l_xyz[2]))), rpy = rot_pjx)
            origin=etree.SubElement(joint, 'origin', xyz = xyz_pjx, rpy= "0 0 0")
            parent_link=etree.SubElement(joint, 'parent', link = 'virt_Y%i' % i)
            child_link=etree.SubElement(joint, 'child', link = 'cable%i' % i)
            axe=etree.SubElement(joint, 'axis', xyz = '%f %f %f' % tuple(-R[:3,2]))
            limit=etree.SubElement(joint, 'limit',lower = str(-0.5*l), upper = str(0.5*l), effort=str(config.joints.passive.effort), velocity=str(config.joints.passive.velocity))
            dynamics=etree.SubElement(joint, 'limit', damping=str(config.joints.passive.damping)) 
            origin=etree.SubElement(joint, 'origin', xyz = "0 0 0", rpy=rot_pjx)
            # rotation cable/pf X



            link = etree.SubElement(robot, 'link', name= 'virt_Xpf%i' % i)
            inertial = etree.SubElement(link, 'inertial')
            l_7 = ('%f %f %f %f %f %f' % tuple(list(pp.reshape(3)) + rpy_gen)).split(" ")
            print(l_7)
            xyz = str(l_7[0]+" "+l_7[1]+" "+l_7[2])
            rot = str(l_7[3]+" "+l_7[4]+" "+l_7[5])
            origin=etree.SubElement(inertial, 'origin', xyz = xyz, rpy = rot)
            mass=etree.SubElement(inertial, 'mass', value = "0.001")
            inertia=etree.SubElement(inertial, 'inertia', ixx="0.001", ixy="0", ixz="0", iyy="0.001", iyz="0", izz="0.001")
            visual = etree.SubElement(link, 'visual')
            origin=etree.SubElement(visual, 'origin', xyz = xyz, rpy = rot)
            geometry=etree.SubElement(visual, 'geometry')
            cylinder=etree.SubElement(geometry, 'box',size = "0.1 0.1 0.1")
            material=etree.SubElement(visual, 'material', name = "${cable_color}")



            # revolute joint around X
            joint = etree.SubElement(robot, 'joint', name= 'rev_Xpf%i' % i)
            joint.set("type", "revolute")
            l_8 = ('0 0 0 %f %f %f' % tuple(rpy_gen)).split(" ")
            xyz_rjx = str(l_8[0]+" "+l_8[1]+" "+l_8[2])
            rot_rjx = str(l_8[3]+" "+l_8[4]+" "+l_8[5])
            origin_xpf=etree.SubElement(joint, 'origin', xyz = "0 0 0", rpy ="0 0 0")
            parent_link=etree.SubElement(joint, 'parent', link = 'platform')
            child_link=etree.SubElement(joint, 'child', link = 'virt_Xpf%i' % i)
            axe=etree.SubElement(joint, 'axis', xyz = '1 0 0')
            limit=etree.SubElement(joint, 'limit', effort=str(config.joints.passive.effort), velocity=str(config.joints.passive.velocity))
            dynamics=etree.SubElement(joint, 'limit', damping=str(config.joints.passive.damping))


            # rotation cable/pf Y
            link = etree.SubElement(robot, 'link', name= 'virt_Ypf%i' % i)
            l_10 = ('%f %f %f %f %f %f' % tuple(list(pp.reshape(3)) + rpy_gen)).split(" ")
            inertial = etree.SubElement(link, 'inertial')
            xyz = str(l_10[0]+" "+l_10[1]+" "+l_10[2])
            rpy_pf = str(l_10[3]+" "+l_10[4]+" "+l_10[5])
            origin_ypf=etree.SubElement(inertial, 'origin', xyz = "0 0 0", rpy = "0 0 0")
            mass=etree.SubElement(inertial, 'mass', value = "0.001")
            inertia=etree.SubElement(inertial, 'inertia', ixx="0.001", ixy="0", ixz="0", iyy="0.001", iyz="0", izz="0.001")
            origin=etree.SubElement(visual, 'origin', xyz = xyz, rpy = rpy_pf)
            geometry=etree.SubElement(visual, 'geometry')
            cub=etree.SubElement(geometry, 'box',size = "0.1 0.1 0.1")
            material=etree.SubElement(visual, 'material', name = "${cable_color}")

            """
            # revolute joint around Y
            joint = etree.SubElement(robot, 'joint', name= 'rev_Ypf%i' % i)
            joint.set("type", "revolute") 
            l_11 = ('0 0 0 %f %f %f' % tuple(rpy_gen)).split(" ")
            xyz_rjx = "0 0 0"
            rot_rjx = str(l_11[3]+" "+l_11[4]+" "+l_11[5])
            origin=etree.SubElement(joint, 'origin', xyz = "0 0 0", rpy = "0 0 0")
            parent_link=etree.SubElement(joint, 'parent', link = 'virt_Xpf%i' % i)
            child_link=etree.SubElement(joint, 'child', link ='virt_Ypf%i' % i)
            axe=etree.SubElement(joint, 'axis', xyz = '0 1 0')
            limit=etree.SubElement(joint, 'limit', effort=str(config.joints.passive.effort), velocity=str(config.joints.passive.velocity))
            dynamics=etree.SubElement(joint, 'limit', damping=str(config.joints.passive.damping))
            """


            # rotation cable/pf Z
            # revolute joint around Z
            joint = etree.SubElement(robot, 'joint', name= 'rev_Zpf%i' % i)
            joint.set("type", "revolute")
            l_9 = ('0 0 0 %f %f %f' % tuple(rpy_gen)).split(" ")
            xyz_rjx = "0 0 0"
            rot_rjx = str(l_9[3]+" "+l_9[4]+" "+l_9[5])
            origin=etree.SubElement(joint, 'origin', xyz = "0 0 "+str(l/2), rpy = "0 0 0")
            parent_link=etree.SubElement(joint, 'parent', link =  'cable%i'% i)
            child_link=etree.SubElement(joint, 'child', link = 'virt_Ypf%i' % i)
            axe=etree.SubElement(joint, 'axis', xyz = '0 0 1')
            limit=etree.SubElement(joint, 'limit', effort=str(config.joints.passive.effort), velocity=str(config.joints.passive.velocity))
            dynamics=etree.SubElement(joint, 'limit', damping=str(config.joints.passive.damping)) 

            gazebo=etree.SubElement(robot, 'gazebo')
            joint = etree.SubElement(gazebo, 'joint', name= 'rev_Ypf%i' % i)
            joint.set("type", "revolute")
            CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy_gen))
            CreateNested(joint, 'parent', 'virt_Xpf%i' % i)
            CreateNested(joint, 'child', 'virt_Ypf%i' % i)
            CreateNested(joint, 'axis/xyz', '0 1 0')
            CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
            CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
            CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)

            # SDF materials

            gazebo=etree.SubElement(robot, 'gazebo')
            gazebo.set("reference","cable%i" % i)
            CreateNested(gazebo, 'material', 'Gazebo/Red')

            

            # SDF materials

            gazebo=etree.SubElement(robot, 'gazebo')
            gazebo.set("reference","cable%i" % i)
            CreateNested(gazebo, 'material', 'Gazebo/Red')


            # write file
            WriteXACRO(robot, name+'.xacro')

