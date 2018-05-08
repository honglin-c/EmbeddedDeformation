#version 330 core
struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

struct Light {
    vec3 position;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

in vec3 Normal;
in vec3 FragPos;

out vec4 color;

uniform vec3 viewPos;
uniform Material material;
uniform Light light;

void main() {
    // Using the half-vector approach (Blinn-Phong Reflection Model) to compute the specular component.
	vec3 viewDir = normalize(viewPos - FragPos);
	vec3 norm = normalize(Normal);
	vec3 lightDir = normalize(light.position - FragPos);
	vec3 halfDir = normalize(viewDir + lightDir);

	float N_dot_L = max(0.0, dot(norm, lightDir));
	float N_dot_H = max(0.0, dot(norm, halfDir));

	float pf = (N_dot_H == 0.0) ? 0.0 : pow(N_dot_H, material.shininess);

	// Ambient
	vec3 ambient = light.ambient * material.ambient;
	// Diffuse
	vec3 diffuse = light.diffuse * material.diffuse * N_dot_L;
	// Specular
	vec3 specular = light.specular * material.specular * pf;

	color = vec4(ambient + diffuse + specular, 1.0f);
}
