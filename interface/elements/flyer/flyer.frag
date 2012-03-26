#version 150

out vec4 out_Color;
in vec3 v_Color;
in vec3 v_transformedNormal;
in vec2 v_TexCoord;
uniform float t;

void main(void)
{
    const vec3 light = vec3(0.58, 0.58, 0.58);
    float shade = clamp(dot(normalize(v_transformedNormal), light),0,1) + 0.2;
    out_Color = vec4(shade, shade, shade, 1.0);
}
