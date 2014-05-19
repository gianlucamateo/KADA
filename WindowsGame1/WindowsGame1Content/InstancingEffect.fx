float4x4 WVP;

 
struct InstancingVSinput
{
 float4 Position : POSITION0;
 float2 TexCoord : TEXCOORD0;
};
 
struct InstancingVSoutput
{
 float4 Position : POSITION0;
 float2 TexCoord : TEXCOORD0;
 float4 Color	 : COLOR0;
};
 
InstancingVSoutput InstancingVS(InstancingVSinput input, float3 instanceTransform : TEXCOORD1, float scale : TEXCOORD2, float3 color : COLOR0)
{
 InstancingVSoutput output;
 float4 pos = input.Position;
 pos = 2 * pos;
 pos = pos+float4(instanceTransform,0);
 //pos.z = pos.z+scale;
 //scale = sqrt(pow(scale,2)+pow(pos.x,2))/1000;
 //float planeScaling = -scale / 670.63;
 //pos.x *= planeScaling;
 //pos.y *= planeScaling;
 
 pos = mul(pos, WVP);
 
 
 output.Position = pos;//float4(input.Position + color,0);
 output.TexCoord = input.TexCoord;
 //color = normalize(color);
 output.Color = float4(color,scale);//float4(1,1,1,0)*(max(max(color.x,color.y),color.z)-min(min(color.x,color.y),color.z))/max(max(color.x,color.y),color.z);//color;//normalize(color);
 return output;
}
 
float4 InstancingPS(InstancingVSoutput input) : COLOR0
{
return input.Color;
 //return float4(input.Color,0);
}
 
technique Instancing
{
 pass Pass0
 {
 AlphaBlendEnable = TRUE;
 DestBlend = INVSRCALPHA;
 SrcBlend = SRCALPHA;
 VertexShader = compile vs_3_0 InstancingVS();
 PixelShader = compile ps_3_0 InstancingPS();
 }
}