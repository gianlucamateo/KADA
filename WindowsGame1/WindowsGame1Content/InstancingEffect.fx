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
 
InstancingVSoutput InstancingVS(InstancingVSinput input, float4x4 instanceTransform : TEXCOORD1, float3 Color : COLOR0)
{
 InstancingVSoutput output;
 float4 pos = input.Position;
 pos = mul(pos, transpose(instanceTransform));
 pos = mul(pos, WVP);
 output.Position = pos;
 output.TexCoord = input.TexCoord;
 output.Color = Color;
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