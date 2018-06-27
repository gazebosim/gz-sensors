set (files
  depth_fragment_shader.glsl
  depth_vertex_shader.glsl
  fragment_shader.glsl
  vertex_shader.glsl
)

install(FILES ${files} DESTINATION ${IGN_SENSORS_PLUGIN_PATH}/media/materials/programs/)
