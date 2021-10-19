import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import shutil

class RandomForestGenerator:

    def __init__(self,xrange,yrange,rrange,lrange,
            cyl_template='../../templates/model.world',
            world_template='../../templates/template.world',
            density=2000):
        self.xrange = xrange
        self.yrange = yrange
        self.rrange = rrange
        self.lrange = lrange
        self.model_tree = ET.parse(cyl_template)
        self.world_template = world_template
        self.density = density

    def cyl2xml(self,c,i):

        volume = np.pi * c[3]**2 * c[4]
        m = volume / self.density
        ixx = 1/12 * m * (3*c[3]**2+c[4]**2)
        izz = 1/2 * m * c[3]**2

        root = self.model_tree.getroot()
        root.set('name','Cylinder %d'%i)

        # Set inertial properties
        mass = root.find('link/inertial/mass')
        mass.text = '%.5f'%m
        ix = root.find('link/inertial/inertia/ixx'); ix.text = '%.5f'%ixx
        iy = root.find('link/inertial/inertia/iyy'); iy.text = '%.5f'%ixx
        iz = root.find('link/inertial/inertia/izz'); iz.text = '%.5f'%ixx

        # Set visual and collision properties
        for x in ['collision','visual']:
            length = root.find('link/%s/geometry/cylinder/length'%x)
            length.text = '%.5f'%c[4]
            radius = root.find('link/%s/geometry/cylinder/radius'%x)
            radius.text = '%.5f'%c[3]

        # Set pose
        pose = root.find('pose')
        pose.text = '%.5f %.5f %.5f 0 0 0'%(c[0],c[1],c[2])

        return ET.tostring(root,encoding='unicode')

    def genWorldFile(self,num_obs,outfile):
        shutil.copy(self.world_template,outfile)
        file = open(outfile,'a')
        cyls = self.genCyls(num_obs)
        self.plotCircles(cyls)
        for (i,c) in enumerate(cyls):
            s = self.cyl2xml(c,i)
            file.write(s)
        file.write('</world>\n</sdf>')
        file.close()

    def genCyls(self,num_obs):
        '''
        Generate num_obs random cylinders within bounds
        cylinders[i] = [xi,yi,zi,ri,li]
        '''
        cylinders = -1*np.ones((num_obs,5))
        i = 0
        while i < num_obs:
            x = np.random.uniform(self.xrange[0],self.xrange[1])
            y = np.random.uniform(self.yrange[0],self.yrange[1])
            r = np.random.uniform(self.rrange[0],self.rrange[1])
            l = np.random.uniform(self.lrange[0],self.lrange[1])
            c = [x,y,l/2,r,l]
            if not self.cylCollides(c,cylinders,i):
                cylinders[i,:] = c
                i += 1
        return cylinders

    def cylCollides(self,cyl,cyls,i):
        '''
        Check if cyl will collide with any cylinder in cyls
        '''
        for j in range(i):
            c = cyls[j]
            if c[3] == -1:
                break
            dist = np.sqrt(sum([(c[i]-cyl[i])**2 for i in range(2)]))
            if dist < c[3]+cyl[3]+0.1:
                return True
        return False

    def plotCircles(self,cyls):
        maxr = max(cyls[:,3])
        xrange = (min(cyls[:,0])-maxr,max(cyls[:,0])+maxr)
        yrange = (min(cyls[:,1])-maxr,max(cyls[:,1])+maxr)

        ax = plt.gca()
        ax.cla()
        ax.set_xlim(xrange)
        ax.set_ylim(yrange)
        ax.set_aspect('equal')
        for c in cyls:
            ax.add_patch(plt.Circle(c[:2],c[3],fill=False))
        plt.show()

if __name__ == "__main__":
    gen = RandomForestGenerator([0,50],[0,50],[0.25,2],[2,10])
    gen.genWorldFile(150,'test.world')
