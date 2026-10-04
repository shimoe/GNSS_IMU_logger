package com.example.gnss_imu_logger.playback
import java.io.File
internal class CsvRowReader(private val file: File) {
 fun forEachRow(block:(Map<String,String>)->Unit) { file.bufferedReader().use { r -> val h=parse(r.readLine()?:error("${file.name}に見出しがありません")); r.lineSequence().filter{it.isNotBlank()}.forEach { l -> val v=parse(l); block(h.indices.associate{h[it] to v.getOrElse(it){""}}) } } }
 private fun parse(s:String):List<String>{val o=mutableListOf<String>();val b=StringBuilder();var q=false;var i=0;while(i<s.length){val c=s[i];when{c=='"'&&q&&i+1<s.length&&s[i+1]=='"'->{b.append('"');i++};c=='"'->q=!q;c==','&&!q->{o+=b.toString();b.clear()};else->b.append(c)};i++};o+=b.toString();return o}
}
