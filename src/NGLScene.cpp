#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/Camera.h>
#include <ngl/Light.h>
#include <ngl/Material.h>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include<perlinnoise.h>



//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for x/y translation with mouse movement
//----------------------------------------------------------------------------------------------------------------------
const static float INCREMENT=0.01;
//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for the wheel zoom
//----------------------------------------------------------------------------------------------------------------------
const static float ZOOM=1;



std::vector<ngl::Vec3> particlePositions;
std::vector<ngl::Vec2> tmpParticlePositions;



// Create a PerlinNoise object with a random permutation vector generated with seed
unsigned int seed = 237;
PerlinNoise pn(seed);

void computeCurl(float	x,	float	y, float &curla, float &curlb)
{
                float	eps	=	1.0;
                float	n1,	n2,	a,	b;

                n1	=	pn.noise(x,	y	+	eps, 0.5);
                n2	=	pn.noise(x,	y	-	eps, 0.5);
                a	=	(n1	-	n2)/(2	*	eps, 0.5);
                n1	=	pn.noise(x	+	eps,	y, 0.5);
                n2	=	pn.noise(x	-	eps,	y, 0.5);
                b	=	(n1	-	n2)/(2	*	eps, 0.5);

//                ofVec2f	curl	=	ofVec2f(a,	-b);

                curla=a;
                curlb=-b;
}



//https://github.com/cabbibo/glsl-curl-noise/blob/master/curl.glsl
ngl::Vec3 snoiseVec3( ngl::Vec3 x ){

  float s  = pn.noise(x.m_x, x.m_y, x.m_z );
  float s1 = pn.noise( x.m_y - 19.1 , x.m_z + 33.4 , x.m_x + 47.2 );
  float s2 = pn.noise( x.m_z + 74.2 , x.m_x - 124.5 , x.m_y + 99.4 );
  ngl::Vec3 c = ngl::Vec3( s , s1 , s2 );
  return c;

}


ngl::Vec3 curlNoise( ngl::Vec3 p ){

  const float e = .1;
  ngl::Vec3 dx = ngl::Vec3( e   , 0.0 , 0.0 );
  ngl::Vec3 dy = ngl::Vec3( 0.0 , e   , 0.0 );
  ngl::Vec3 dz = ngl::Vec3( 0.0 , 0.0 , e   );

  ngl::Vec3 p_x0 = snoiseVec3( p - dx );
  ngl::Vec3 p_x1 = snoiseVec3( p + dx );
  ngl::Vec3 p_y0 = snoiseVec3( p - dy );
  ngl::Vec3 p_y1 = snoiseVec3( p + dy );
  ngl::Vec3 p_z0 = snoiseVec3( p - dz );
  ngl::Vec3 p_z1 = snoiseVec3( p + dz );

  float x = p_y1.m_z - p_y0.m_z - p_z1.m_y + p_z0.m_y;
  float y = p_z1.m_x - p_z0.m_x - p_x1.m_z + p_x0.m_z;
  float z = p_x1.m_y - p_x0.m_y - p_y1.m_x + p_y0.m_x;

  const float divisor = 1.0 / ( 2.0 * e );
  ngl::Vec3 retvec( x , y , z );
//  retvec.normalize();
  return ( retvec * divisor );

}



NGLScene::NGLScene()
{
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  m_rotate=false;
  // mouse rotation values set to 0
  m_spinXFace=0;
  m_spinYFace=0;
  setTitle("VAOPrimitives Demo");
  m_animate=true;
}


NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL(QResizeEvent *_event)
{
  m_width=_event->size().width()*devicePixelRatio();
  m_height=_event->size().height()*devicePixelRatio();
  // now set the camera size values as the screen size has changed
  m_cam.setShape(45.0f,(float)width()/height(),0.05f,350.0f);
}

void NGLScene::resizeGL(int _w , int _h)
{
  m_cam.setShape(45.0f,(float)_w/_h,0.05f,350.0f);
  m_width=_w*devicePixelRatio();
  m_height=_h*devicePixelRatio();
}


void NGLScene::initializeGL()
{
  // we need to initialise the NGL lib which will load all of the OpenGL functions, this must
  // be done once we have a valid GL context but before we call any GL commands. If we dont do
  // this everything will crash
  ngl::NGLInit::instance();
  glClearColor(0.4f, 0.4f, 0.4f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // now to load the shader and set the values
  // grab an instance of shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  // we are creating a shader called Phong
  shader->createShaderProgram("Phong");
  // now we are going to create empty shaders for Frag and Vert
  shader->attachShader("PhongVertex",ngl::ShaderType::VERTEX);
  shader->attachShader("PhongFragment",ngl::ShaderType::FRAGMENT);
  // attach the source
  shader->loadShaderSource("PhongVertex","shaders/PhongVertex.glsl");
  shader->loadShaderSource("PhongFragment","shaders/PhongFragment.glsl");
  // compile the shaders
  shader->compileShader("PhongVertex");
  shader->compileShader("PhongFragment");
  // add them to the program
  shader->attachShaderToProgram("Phong","PhongVertex");
  shader->attachShaderToProgram("Phong","PhongFragment");
  // now bind the shader attributes for most NGL primitives we use the following
  // layout attribute 0 is the vertex data (x,y,z)
  shader->bindAttribute("Phong",0,"inVert");
  // attribute 1 is the UV data u,v (if present)
  shader->bindAttribute("Phong",1,"inUV");
  // attribute 2 are the normals x,y,z
  shader->bindAttribute("Phong",2,"inNormal");

  // now we have associated this data we can link the shader
  shader->linkProgramObject("Phong");
  // and make it active ready to load values
  (*shader)["Phong"]->use();
    shader->setShaderParam1i("Normalize",1);
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0,8,56);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_cam.set(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_cam.setShape(45,(float)720.0/576.0,0.05,350);
  shader->setShaderParam3f("viewerPos",m_cam.getEye().m_x,m_cam.getEye().m_y,m_cam.getEye().m_z);

  // now pass the modelView and projection values to the shader
  // the shader will use the currently active material and light0 so set them
  ngl::Material m(ngl::STDMAT::GOLD);
  m.loadToShader("material");
  m_lightAngle=0.0;
  m_light.reset( new ngl::Light(ngl::Vec3(sin(m_lightAngle),2,cos(m_lightAngle)),ngl::Colour(1,1,1,1),ngl::Colour(1,1,1,1),ngl::LightModes::DIRECTIONALLIGHT));

  // now create our light this is done after the camera so we can pass the
  // transpose of the projection matrix to the light to do correct eye space
  // transformations
  ngl::Mat4 iv=m_cam.getViewMatrix();
  iv.transpose();
  m_light->setTransform(iv);
  // load these values to the shader as well
  m_light->loadToShader("light");


  ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
  prim->createSphere("sphere",0.5,50);

  prim->createCylinder("cylinder",0.5,1.4,40,40);

  prim->createCone("cone",0.5,1.4,20,20);

  prim->createDisk("disk",0.8,120);
  prim->createTorus("torus",0.15,0.4,40,40);
  prim->createTrianglePlane("plane",14,14,80,80,ngl::Vec3(0,1,0));
  // as re-size is not explicitly called we need to do this.
  glViewport(0,0,width(),height());
  // this timer is going to trigger an event every 40ms which will be processed in the
  //
  m_lightTimer =startTimer(40);



  for(int i=-5;i<5;i++)
  {
      for(int j=-5;j<5;j++)
      {
          for(int k=-5;k<5;k++)
          {
             particlePositions.push_back(ngl::Vec3(2*i,2*j,2*k));
          }
      }
  }

  tmpParticlePositions.resize(particlePositions.size());


}


void NGLScene::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;
  M=m_transform.getMatrix()*m_mouseGlobalTX;
  MV=  M*m_cam.getViewMatrix();
  MVP=  MV*m_cam.getProjectionMatrix();
  normalMatrix=MV;
  normalMatrix.inverse();
  shader->setShaderParamFromMat4("MV",MV);
  shader->setShaderParamFromMat4("MVP",MVP);
  shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
  shader->setShaderParamFromMat4("M",M);
}

void NGLScene::drawScene(const std::string &_shader)
{
  // grab an instance of the shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)[_shader]->use();
  // clear the screen and depth buffer
  // Rotation based on the mouse position for our global
  // transform
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  // create the rotation matrices
  rotX.rotateX(m_spinXFace);
  rotY.rotateY(m_spinYFace);
  // multiply the rotations
  m_mouseGlobalTX=rotY*rotX;
  // add the translations
  m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
  m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;

   // get the VBO instance and draw the built in teapot
  ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();

//  m_transform.reset();
//  {
//    loadMatricesToShader();
//    prim->draw("teapot");
//  } // and before a pop





//  m_transform.reset();
//  {
//      //draw the thing
//      for(int i=0;i<particlePositions.size();i++)
//      {

//        m_transform.setPosition(particlePositions[i].m_x, particlePositions[i].m_y, 0);
//        loadMatricesToShader();
////        prim->draw("teapot");
//      }
//  }

  //copy & fluctuate & copy back so as to draw at the next iteration
  //std::copy(particlePositions.begin(), particlePositions.end(), tmpParticlePositions.begin());


  for(int i=0;i<particlePositions.size();i++)
  {
    m_transform.setPosition(particlePositions[i].m_x, particlePositions[i].m_y, particlePositions[i].m_z);
    loadMatricesToShader();
    prim->draw("sphere");
  }
  //std::copy(tmpParticlePositions.begin(), tmpParticlePositions.end(), particlePositions.begin());






}

void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_width,m_height);
  drawScene("Phong");

}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseMoveEvent (QMouseEvent * _event)
{
  // note the method buttons() is the button state when event was called
  // this is different from button() which is used to check which button was
  // pressed when the mousePress/Release event is generated
  if(m_rotate && _event->buttons() == Qt::LeftButton)
  {
    int diffx=_event->x()-m_origX;
    int diffy=_event->y()-m_origY;
    m_spinXFace += (float) 0.5f * diffy;
    m_spinYFace += (float) 0.5f * diffx;
    m_origX = _event->x();
    m_origY = _event->y();
    update();

  }
        // right mouse translate code
  else if(m_translate && _event->buttons() == Qt::RightButton)
  {
    int diffX = (int)(_event->x() - m_origXPos);
    int diffY = (int)(_event->y() - m_origYPos);
    m_origXPos=_event->x();
    m_origYPos=_event->y();
    m_modelPos.m_x += INCREMENT * diffX;
    m_modelPos.m_y -= INCREMENT * diffY;
    update();

   }
}


//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mousePressEvent ( QMouseEvent * _event)
{
  // this method is called when the mouse button is pressed in this case we
  // store the value where the maouse was clicked (x,y) and set the Rotate flag to true
  if(_event->button() == Qt::LeftButton)
  {
    m_origX = _event->x();
    m_origY = _event->y();
    m_rotate =true;
  }
  // right mouse translate mode
  else if(_event->button() == Qt::RightButton)
  {
    m_origXPos = _event->x();
    m_origYPos = _event->y();
    m_translate=true;
  }

}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseReleaseEvent ( QMouseEvent * _event )
{
  // this event is called when the mouse button is released
  // we then set Rotate to false
  if (_event->button() == Qt::LeftButton)
  {
    m_rotate=false;
  }
        // right mouse translate mode
  if (_event->button() == Qt::RightButton)
  {
    m_translate=false;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::wheelEvent(QWheelEvent *_event)
{

	// check the diff of the wheel position (0 means no change)
	if(_event->delta() > 0)
	{
		m_modelPos.m_z+=ZOOM;
	}
	else if(_event->delta() <0 )
	{
		m_modelPos.m_z-=ZOOM;
	}
	update();
}
//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  // turn on wirframe rendering
  case Qt::Key_W : glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); break;
  // turn off wire frame
  case Qt::Key_S : glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); break;
  // show full screen
  case Qt::Key_F : showFullScreen(); break;
  // show windowed
  case Qt::Key_N : showNormal(); break;
  default : break;
  }
  // finally update the GLWindow and re-draw
  if (isExposed())
    update();
}


bool NGLScene::sphereToSphereCollide(const ngl::Vec3 & p1, float teapotRadius)
{
    for (int i =0; i<particlePositions.size();i++)
    {
        if ( (particlePositions[i]-p1).length() < teapotRadius && (p1-particlePositions[i]).length()!=0) //(p1-particlePositions[i]).length()!=0 is the check for itself as I have stupidly & intentionally chose to avoid assinging IDs to particles
            return true;
        else
            return false;
    }

}

ngl::Vec3 constvel(0,10,0);

void NGLScene::updateLight()
{
//  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

//  (*shader)["Phong"]->use();
//  // change the light angle
//  m_lightAngle+=0.1;

//  // now set this value and load to the shader
//  m_light->setPosition(ngl::Vec3(4.0*cos(m_lightAngle),2,4.0*sin(m_lightAngle)));
//  m_light->loadToShader("light");


    for(int i=0;i<particlePositions.size();i++)
    {
    //      float curla=0; float curlb=0;
    //      computeCurl(particlePositions[i].m_x, particlePositions[i].m_y, curla, curlb);
    //      ngl::Vec2 Velocity(curla, curlb) ;


          ngl::Vec3 noiseVel=curlNoise(ngl::Vec3(particlePositions[i].m_x, particlePositions[i].m_y, particlePositions[i].m_z) );
//          constvel.m_y+=1;


          ngl::Vec3 vel(noiseVel.m_x/25, /*cos(ngl::radians*/constvel.m_y*noiseVel.m_y/25, noiseVel.m_z/25);

          float sphereradius=0.5;

          if ( sphereToSphereCollide(particlePositions[i],sphereradius) )
          {
              particlePositions[i].m_x-=sphereradius;
              particlePositions[i].m_y-=sphereradius;
              particlePositions[i].m_y-=sphereradius;

              vel=-vel;
          }

          particlePositions[i].m_x=particlePositions[i].m_x+(vel.m_x);
          particlePositions[i].m_y=particlePositions[i].m_y+(vel.m_y);
          particlePositions[i].m_z=particlePositions[i].m_z+(vel.m_z);

    }

}

void NGLScene::timerEvent(QTimerEvent *_event )
{
// if the timer is the light timer call the update light method
  if(_event->timerId() == m_lightTimer && m_animate==true)
  {
    updateLight();
  }
    // re-draw GL
update();
}

