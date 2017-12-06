varying vec4 point;

void main()
{
  // TODO replace projection matrix with one that projects half ring into a cube
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  // Vertex in world space
  point = gl_ModelViewMatrix * gl_Vertex;
}
