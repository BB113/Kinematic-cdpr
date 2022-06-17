#!/usr/bin/python3

from mod_create_2 import *
import yaml
import sys
import numpy as np
from math import *
import transformations as tr
from os.path import exists

if __name__ == '__main__':
    fichier = open("cable_position", "w")
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

    # XACRO building
    robot = etree.Element('robot')

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
            rpy = list(tr.euler_from_matrix(R))
            # rpy of z-axis
            # cable position to stick to the platform
            a = l/(2.*np.linalg.norm(pp-fp))
            cp = list((pp - a*(pp-fp)).reshape(3))
            # create cable
            link = etree.SubElement(robot, 'link', name= 'cable%i' % i)

            collision = etree.SubElement(link, 'collision', name= 'cable%i' % i)
            l_xyz = '%f %f %f %f %f %f' % tuple(cp + rpy)
            print(l_xyz.split(" "))
            xyz = str(l_xyz.split(" ")[0]+" "+l_xyz.split(" ")[1]+" "+l_xyz.split(" ")[2])
            rot = str(l_xyz.split(" ")[3]+" "+l_xyz.split(" ")[4]+" "+l_xyz.split(" ")[5])

            origin=etree.SubElement(collision, 'origin', xyz = xyz, rpy = rot)
            geometry=etree.SubElement(collision, 'geometry')
            cylinder=etree.SubElement(geometry, 'cylinder',radius = str(config.cable.radius), lenght = str(l))
            # write file
            fichier.write(str('%f %f %f %f %f %f' % tuple(cp + rpy)+"\n"))

            visual = etree.SubElement(link, 'visual', name= 'cable%i' % i)

            origin=etree.SubElement(visual, 'origin', xyz = xyz, rpy = rot)
            geometry=etree.SubElement(visual, 'geometry')
            cylinder=etree.SubElement(geometry, 'cylinder',radius = str(config.cable.radius), lenght = str(l))
            material=etree.SubElement(visual, 'material', name = "${cable_color}")

            
            # write file

    WriteXACRO(robot, name+'.xacro')