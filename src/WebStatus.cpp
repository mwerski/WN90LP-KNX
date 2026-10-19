#include "WebStatus.h"
#include "OpenKNX.h"

#ifdef OPENKNX_WEBSERVER
    #include "NetworkModule.h"
using OpenKNX::Network::WEB_GET;
using OpenKNX::Network::WebRequest;
using OpenKNX::Network::WebResponse;

// Seite im OpenKNX-Layout; Texte als HTML-Entities, damit der Quelltext ASCII bleibt.
// Die Werte rendert das Skript aus /weather.json (nur textContent, kein innerHTML mit Daten).
static const char PAGE[] = R"HTML(<div class='container'>
<h2>Wetterstation</h2>
<p id='wx-state'>Lade Daten&hellip;</p>
<div id='wx-groups'></div>
<style>
#wx-groups table{width:100%;table-layout:fixed;border-collapse:collapse;margin-bottom:1.2em}
#wx-groups col.n{width:56%}
#wx-groups col.v,#wx-groups col.s{width:22%}
#wx-groups h3{margin:1.2em 0 .3em}
#wx-groups td{overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
#wx-groups th{text-align:left;padding:.4em .3em;border-bottom:2px solid #8884}
#wx-groups th.v,#wx-groups th.s{text-align:right}
#wx-groups td{padding:.25em .3em;border-bottom:1px solid #8882}
#wx-groups td.v{text-align:right;white-space:nowrap;font-variant-numeric:tabular-nums}
#wx-groups td.s{text-align:right;white-space:nowrap;opacity:.6}
#wx-groups td.txt{white-space:normal;overflow:visible;text-overflow:clip}
#wx-groups tr.off{opacity:.45}
#wx-groups tr.inv td.v{opacity:.5}
.wx-bad{color:#c33;font-weight:bold}
</style>
<script>
(function(){
var SYMBOL=['sonnig','bew\u00f6lkt','Schauer','starker Regen','Gewitter','hoher UV-Index'];
var DN=['Tag','Nacht'];
var BOOL={raining:['nein','ja'],night:DN,night_sun:DN,night_civil:DN,night_nautical:DN,night_astro:DN,summer:['Winter','Sommer'],sun_falling:['steigt','f\u00e4llt']};
function num(v,d){return v.toLocaleString('de-DE',{minimumFractionDigits:d,maximumFractionDigits:d});}
function fmt(x,v,t){
  if(v===null||v===undefined)return '\u2013';
  if(t!==undefined)return t;
  if(x.f===3){var m=Math.round(v/60);return Math.floor(m/60)+':'+('0'+m%60).slice(-2)+' h';}
  if(x.k==='weather_symbol')return SYMBOL[v]||String(v);
  if(x.b){var m=BOOL[x.k]||['nein','ja'];return m[v?1:0];}
  return num(v,x.d)+(x.u?' '+x.u:'');
}
function el(t,c,txt){var e=document.createElement(t);if(c)e.className=c;if(txt!==undefined)e.textContent=txt;return e;}
function render(j){
  var st=document.getElementById('wx-state');
  if(!j.values){st.textContent='Warte auf Daten\u2026';return false;}
  st.textContent='';
  var parts=[['WN90LP',j.wn90],['SDS011',j.sds],['1-Wire',j.ow],['InfluxDB',j.influx],['DWD',j.dwd]];
  parts.forEach(function(p){
    if(p[1]===undefined)return;
    var s=el('span',(p[1]==='ok'||p[1]==='wartet')?'':'wx-bad',p[0]+': '+p[1]);st.appendChild(s);st.appendChild(document.createTextNode(' \u00b7 '));
  });
  st.appendChild(document.createTextNode('Uhrzeit: '+(j.time?'g\u00fcltig':'unbekannt')+' \u00b7 MQTT: '+(j.mqtt?'verbunden':'aus')+
    ' \u00b7 WLAN '+j.rssi+' dBm \u00b7 Laufzeit '+Math.floor(j.up/3600)+' h \u00b7 Firmware '+j.fw));
  // Neuen Inhalt vollstaendig aufbauen und in einem Schritt austauschen (kein Flackern)
  var old=document.getElementById('wx-groups'),box=el('div');box.id='wx-groups';
  if(j.dwd_headline){box.appendChild(el('p','wx-bad',j.dwd_headline));}
  var groups={},order=[];
  j.values.forEach(function(x){if(!groups[x.g]){groups[x.g]=[];order.push(x.g);}groups[x.g].push(x);});
  order.forEach(function(g){
    box.appendChild(el('h3','',g));
    var t=el('table'),cg=el('colgroup'),h=el('tr');
    ['n','v','s'].forEach(function(c){cg.appendChild(el('col',c));});
    t.appendChild(cg);
    [['n','Wert'],['v','aktuell'],['s','zuletzt gesendet']].forEach(function(c){h.appendChild(el('th',c[0],c[1]));});
    t.appendChild(h);
    groups[g].forEach(function(x){
      var r=el('tr',(x.a?'':'off')+(x.v===null?' inv':''));
      var n=el('td','',x.n+(x.a?'':' (nicht aktiv)'));n.title=n.textContent;r.appendChild(n);
      var tx=(x.t!==undefined||x.ts!==undefined)?' txt':'';
      r.appendChild(el('td','v'+tx,fmt(x,x.v,x.t)));
      r.appendChild(el('td','s'+tx,x.a&&x.s!==null?fmt(x,x.s,x.ts):''));
      t.appendChild(r);
    });
    box.appendChild(t);
  });
  old.parentNode.replaceChild(box,old);
  return true;
}
function poll(){
  fetch('/weather.json',{cache:'no-store'}).then(function(r){return r.json();})
  .then(function(j){setTimeout(poll,render(j)?5000:1000);})
  .catch(function(){document.getElementById('wx-state').textContent='Keine Verbindung';setTimeout(poll,5000);});
}
poll();
})();
</script>
</div>)HTML";

void WebStatus::setup()
{
    openknxNetwork.webserver.addMenuItem("Wetter", "/weather", 10);
    openknxNetwork.webserver.addRoute(WEB_GET, "/weather", [](WebRequest &, WebResponse &res) {
        res.setContentType("text/html");
        res.setLayout(true);
        res.sendStatic(PAGE);
    });
    openknxNetwork.webserver.addRoute(WEB_GET, "/weather.json", [this](WebRequest &, WebResponse &res) {
        _demandUntil = millis() + 15000;
        res.setContentType("application/json");
        // Direkt aus dem Schnappschuss senden (send kopiert selbst), keine weitere Kopie im Speicher
        std::lock_guard<std::mutex> guard(_lock);
        res.send(_json.c_str());
    });
}

bool WebStatus::due()
{
    // Nur solange die Seite abgefragt wird (15 s nach der letzten Anfrage), dann alle 2 s
    if (_demandUntil == 0 || (int32_t)(_demandUntil - millis()) <= 0)
        return false;
    if (!delayCheck(_lastBuild, 2000))
        return false;
    _lastBuild = millis();
    return true;
}

void WebStatus::update(std::string &&json)
{
    // Leer bei Speichermangel: alten Schnappschuss behalten
    if (json.empty())
        return;
    std::lock_guard<std::mutex> guard(_lock);
    _json = std::move(json);
}

#else

void WebStatus::setup() {}
bool WebStatus::due() { return false; }
void WebStatus::update(std::string &&) {}

#endif
