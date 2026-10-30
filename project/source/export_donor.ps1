$ErrorActionPreference='Stop'
$dll='E:/Program Files/SOLIDWORKS Corp/SOLIDWORKS/api/redist'
Add-Type -Path "$dll/SolidWorks.Interop.sldworks.dll"
Add-Type -Path "$dll/SolidWorks.Interop.swconst.dll"
Add-Type -ReferencedAssemblies "$dll/SolidWorks.Interop.sldworks.dll","$dll/SolidWorks.Interop.swconst.dll" -TypeDefinition @'
using System;
using System.IO;
using System.Globalization;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public class DonorExport {
 public static string Run(string src,string dst) {
  Directory.CreateDirectory(dst);
  var a=(ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
  int e=0,w=0;
  var doc=(IModelDoc2)a.OpenDoc6(src+"/1.SLDASM",2,3,"",ref e,ref w);
  if(doc==null)throw new Exception("Open failed "+e);
  ((IAssemblyDoc)doc).ResolveAllLightWeightComponents(false);
  var rows=new System.Text.StringBuilder();
  var comps=(object[])((IAssemblyDoc)doc).GetComponents(false);
  foreach(IComponent2 c in comps){
   rows.Append(c.Name2+"|"+c.GetPathName()+"|"+c.IsSuppressed()+"|");
   var t=(IMathTransform)c.Transform2;
   if(t!=null)foreach(double v in (double[])t.ArrayData)rows.Append(v.ToString("R",CultureInfo.InvariantCulture)+",");
   rows.AppendLine();
  }
  File.WriteAllText(dst+"/components.txt",rows.ToString());
  bool bin=a.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat);
  bool keep=a.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLDontTranslateToPositive);
  int units=a.GetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits);
  try {
   a.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat,true);
   a.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLDontTranslateToPositive,true);
   a.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits,0);
   foreach(string file in Directory.GetFiles(src,"*.SLDPRT")){
    var p=(IModelDoc2)a.OpenDoc6(file,1,3,"",ref e,ref w);
    if(p==null)throw new Exception(file+" failed "+e);
    bool ok=p.Extension.SaveAs(dst+"/"+Path.GetFileNameWithoutExtension(file)+".stl",0,1,null,ref e,ref w);
    rows.AppendLine(file+" export="+ok+" error="+e);
   }
  } finally {
   a.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat,bin);
   a.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLDontTranslateToPositive,keep);
   a.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits,units);
  }
  return rows.ToString();
 }
}
'@
[DonorExport]::Run('E:/零件库/机械爪 (1)/temp','E:/零件库/servo_rack_lift_dual_servo_round_claw/reference/new_gripper')
