"""Verify the captured native records against a supplied read-only source census."""
import argparse,json,sqlite3,zlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--native-sqlite',required=True);a=p.parse_args()
source=Path(a.native_sqlite).resolve();db=sqlite3.connect(source.as_uri()+'?mode=ro',uri=True)
data=json.loads((Path(__file__).parent/'native_source_records.json').read_text())
for e in data['records']:
 row=db.execute('select entity_type,definition_block,native_tags_zlib from records where handle=?',(e['handle'],)).fetchone()
 assert row and row[0]==e['type'] and row[1]==e['definition_block'] and json.loads(zlib.decompress(row[2]))==e['native_tags'],e['handle']
print(json.dumps({'all_native_records_match':True,'record_count':len(data['records']),'sqlite_open_mode':'read_only'}))
