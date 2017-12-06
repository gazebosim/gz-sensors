uniform float maxRange;

varying vec4 point;

void main()
{
  float l = min(maxRange, length(point.xyz));

  float r = 0.0;
  float g = 0.0;
  float b = 0.0;
  float a = 1.0;

  r = l / maxRange;
  g = l / 2.0 / maxRange;
  b = l / 4.0 / maxRange;
  // Currently no pixel format grants access to alpha
  a = l / 8.0 / maxRange;

  gl_FragColor = vec4(r, g, b, a);
}
