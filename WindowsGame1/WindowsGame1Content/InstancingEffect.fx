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
 float3 Color	 : COLOR0;
};
 
InstancingVSoutput InstancingVS(InstancingVSinput input, float3 instanceTransform : TEXCOORD1, float scale : TEXCOORD2, float3 color : COLOR0)
{
 InstancingVSoutput output;
 float4 pos = input.Position;
 pos = pos+float4(instanceTransform,0);
 float4x4 Scale = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1/scale};
 pos = mul(pos, Scale);
 pos = mul(pos, WVP);
 
 
 output.Position = pos;
 output.TexCoord = input.TexCoord;
 output.Color = color/pos.z;
 return output;
}
 
float4 InstancingPS(InstancingVSoutput input) : COLOR0
{
 return float4(input.Color,0);
}
 
technique Instancing
{
 pass Pass0
 {
 VertexShader = compile vs_3_0 InstancingVS();
 PixelShader = compile ps_3_0 InstancingPS();
 }
}