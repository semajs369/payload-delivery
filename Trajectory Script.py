import matplotlib as mpl
#from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import math 
#import time
import sys
#import dronekit

#vehicle = dronekit.connect("/dev/serial0", baud=57600) #connect to the drone

target = np.array([[10,20,0],[5,5,0],[8,9,0]]) #target coordinates FROM MAVLINK

origin = (51.377358,-2.322220) #lat long origin point

for j in range (3) :

        err = 100000

        while err > 500 : #size of error sphere
                
                #Get GPS latlong data
                lat = 51.377358#vehicle.location.global_frame.lat
                lon = -2.322220#vehicle.location.global_frame.lon
                alt = 15#vehicle.location.global_frame.alt
            
                #Convert GPS to cartesian
                xx = (lon-origin[1])*40000*math.cos((lat+origin[0])*math.pi/360)*1000/360
                yy = (origin[0]-lat)*40000*1000/360
                zz = alt

                #Get airspeed, groundspeed and heading
                vair = 20#vehicle.airspeed
                vground = 22#vehicle.groundspeed
                headingD = 12#vehicle.heading       #Degrees from north
                headingR = math.radians(headingD) 
                vzz = 0#vehicle.velocity[2]       #PERHAPS DONT USE THE Z VELOCITY - WONT MAKE BIG DIFFERENCE ANYWAY?
                
                #Convert airspeed, groundspeed and heading to xyz
                [vxxGROUND,vyyGROUND] = [vground*math.sin(headingR),vground*math.cos(headingR)]
                [vxxAIR,vyyAIR] = [vair*math.sin(headingR),vair*math.cos(headingR)]
                
                #Calculate windspeed
                [vxxWIND,vyyWIND] = [(vxxGROUND-vxxAIR),(vyyGROUND-vyyAIR)]

                # INSTANTANIOUS flight parameters cartesian
                vxyzUAS = np.array([vxxGROUND,vyyGROUND,vzz])    #UAS Velocity Vector
                xyzUAS = np.array([xx,yy,zz])  #UAS Position Vector
                vxyzWIND = np.array([vxxWIND,vyyWIND,0])    #Wind Vector assuming z = 0


                # Trajectory parameters:
                rho = 1.225 #density of air
                Apay   = 0.018 #Payload avg surface area
                Apara = 0.2916 #Parachute surface area
                Cpay   = 1.5 #Payload avg drag coefficient
                Cpara = 1.8 #Parachute drag coefficient
                m   = 1 #mass of object
                g = 9.806 #gravity
                delta = 10 #parachute delay lOOK INTO CALCULATING/EXPERIMENTING TO FIND PARACHUTE OPENING TIME
                gamma = 0.5 #xy parachute drag factor (fraction of z)LOOK IN TO LITERATURE
                alpha = 1.2 # wind shear power law

                # Iteration parameters
                dt  = 1 #time steps

                # Initial conditions
                vy = [vxyzUAS[1]]
                vz = [vxyzUAS[2]]
                vx = [vxyzUAS[0]]

                wy = [vxyzWIND[1]]
                wz = [vxyzWIND[2]]
                wx = [vxyzWIND[0]]

                y = [xyzUAS[1]]
                z = [xyzUAS[2]]
                x = [xyzUAS[0]]
                t = [0]

                # Iterate through xyz
                for i in range(sys.maxsize**10):
                    
                        a_Dy = ((Cpay*rho*Apay*vy[i]**2)/(2*m)) + gamma*(0.5*(np.sign(t[-1]-delta)+1)*(Cpara*rho*Apara*vy[i]**2)/(2*m)) #y drag force
                        a_Dz = ((Cpay*rho*Apay*vz[i]**2)/(2*m)) + (0.5*(np.sign(t[-1]-delta) + 1)*(Cpara*rho*Apara*vz[i]**2)/(2*m)) #zdrag force
                        a_Dx = ((Cpay*rho*Apay*vx[i]**2)/(2*m)) + gamma*(0.5*(np.sign(t[-1]-delta) + 1)*(Cpara*rho*Apara*vx[i]**2)/(2*m)) #x drag force

                        aw_Dy = ((Cpay*rho*Apay*wy[i]**2)/(2*m)) + gamma*(0.5*(np.sign(t[-1]-delta) + 1)*(Cpara*rho*Apara*wy[i]**2)/(2*m))#y wind force
                        aw_Dz = ((Cpay*rho*Apay*wz[0]**2)/(2*m)) + (0.5*(np.sign(t[-1]-delta) + 1)*(Cpara*rho*Apara*wz[0]**2)/(2*m)) #z wind force
                        aw_Dx = ((Cpay*rho*Apay*wx[i]**2)/(2*m)) + gamma*(0.5*(np.sign(t[-1]-delta) + 1)*(Cpara*rho*Apara*wx[i]**2)/(2*m)) #x wind force

                        vy.append(vy[i] - np.sign(vy[-1])*a_Dy*dt + np.sign(wy[-1])*aw_Dy*dt)
                        vz.append(vz[i] - g*dt - np.sign(vz[-1])*a_Dz*dt + np.sign(wz[0])*aw_Dz*dt)
                        vx.append(vx[i] - np.sign(vx[-1])*a_Dx*dt + np.sign(wx[-1])*aw_Dx*dt)
                        
                        y.append(y[i] + vy[i]*dt)
                        z.append(z[i] + vz[i]*dt)
                        x.append(x[i] + vx[i]*dt)
                        t.append(t[i] + dt)
                        
                        if z[-1] < 0: #hits ground
                            break
                        
                        wy.append(vxyzWIND[1]*((z[-1]/xyzUAS[2])**alpha)) # wind shear power law
                        wx.append(vxyzWIND[0]*((z[-1]/xyzUAS[2])**alpha)) # wind shear power law
                                                
                        

                # GIVEN CURRENT CONDITIONS, PROJECTED PAYLOAD LANDING COORDINATES

                xyz = np.array([x,y,z])
                xyzlanding = np.array([(xyz[0][-1]),(xyz[1][-1]),(xyz[2][-1])])
                print ('Landing Coordinates:', xyzlanding)

                #PROXIMITY BETWEEN TARGET AND PROJECTION
                xyzrelease = target[j] - xyzlanding
                print ('Target and projection proximity:',xyzrelease)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                #FEEDBACK LOOP SIGNAL TO PIXHAWK TO CONVERGE TARGET AND PROJECTION

                # WOULD THIS BE UNSTABLE?
                # PID CONTROL NEEDED?
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                
                err = np.sqrt(xyzrelease.dot(xyzrelease))


        #Break loop
        #RELEASE COMMAND WHEN PROJECTED = LIVE
        print ('Close Enough: Signal to Pixhawk: Release payload ', j+1)
        print ('Time from release to impact:', t[-1])

# Plot
#fig2 = plt.figure('xy')
#plt.xlabel('x traj')
#plt.ylabel('y traj')
#plt.plot(x,y)
#plt.show()

#fig3 = plt.figure('yz')
#plt.xlabel('y traj')
#plt.ylabel('z traj')
#plt.plot(y,z)
#plt.show()

mpl.rcParams['legend.fontsize'] = 10
fig1 = plt.figure('xyz')
ax = fig1.gca(projection='3d')
ax.plot(x,y,z, label = 'trajectory curve')
#ax.scatter(target[0][0],target[0][1],target[0][2])
#ax.scatter(target[1][0],target[1][1],target[1][2])
#ax.scatter(target[2][0],target[2][1],target[2][2])
#ax.set_xlim([-50,50])
#ax.set_ylim([-50,50])
ax.set_zlim([0,xyzUAS[2]])
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#ax.view_init(elev=90., azim=90)
ax.legend()
plt.show()

#data = open('Data.txt','w')
#data.write(str(x))
#data.write('\n')
#data.write(str(y))
#data.write('\n')
#data.write(str(z))
#data.write('\n')
#data.write(str(t))
#data.close()
