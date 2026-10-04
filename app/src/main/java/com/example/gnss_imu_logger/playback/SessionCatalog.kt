package com.example.gnss_imu_logger.playback
import org.json.JSONObject
import java.io.File
class SessionCatalog(private val documentsDirectory:File){
 fun load()=documentsDirectory.listFiles()?.filter{it.isDirectory}?.map(::read)?.sortedByDescending{it.sessionId}.orEmpty()
 private fun read(d:File):SessionListItem=runCatching{val j=JSONObject(File(d,"session.json").readText());val t=j.optJSONObject("time");val st=j.optJSONObject("statistics");val a=t?.optLong("start_elapsed_ns");val b=t?.takeUnless{it.isNull("end_elapsed_ns")}?.optLong("end_elapsed_ns");SessionListItem(j.optString("session_id",d.name),d,t?.optLong("start_utc_ns"),t?.takeUnless{it.isNull("end_utc_ns")}?.optLong("end_utc_ns"),if(a!=null&&b!=null)(b-a)/1_000_000 else null,j.optString("mode"),j.optString("status"),st?.optJSONObject("gnss")?.optLong("count"),st?.opt("calibration") is JSONObject,File(d,"lean_angle.csv").isFile,File(d,"session.incomplete").exists(),null)}.getOrElse{SessionListItem(d.name,d,null,null,null,"unknown","error",null,false,false,File(d,"session.incomplete").exists(),it.message)}
}
