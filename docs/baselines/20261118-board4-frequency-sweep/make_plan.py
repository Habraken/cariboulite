"""50 MHz base grid with explicit amateur/ISM and RF path anchors."""
import json
from pathlib import Path
bands=[
 ('amateur-6m',50,52,[50.1,51,51.9],'Region 1'),
 ('amateur-4m',70,70.5,[70.05,70.25,70.45],'Region 1, national availability varies'),
 ('amateur-2m',144,146,[144.1,145,145.9],'Region 1'),
 ('amateur-1p25m',222,225,[222.1,223.5,224.9],'Supplementary Region 2'),
 ('amateur-70cm',430,440,[430.1,435,439.9],'Region 1'),
 ('amateur-33cm',902,928,[902.1,915,927.9],'Supplementary Region 2'),
 ('amateur-23cm',1240,1300,[1240.1,1270,1299.9],'Region 1'),
 ('amateur-13cm',2300,2450,[2300.1,2320.1,2360,2399.9,2425,2449.9],'Region 1, national subbands vary'),
 ('amateur-9cm',3400,3410,[3400.1,3405,3409.9],'Region 1, national availability varies'),
 ('amateur-6cm',5650,5850,[5650.1,5750,5849.9],'Region 1'),
 ('ISM-40MHz',40.66,40.70,[40.67,40.68,40.69],'ITU 5.150'),
 ('ISM-433MHz',433.05,434.79,[433.1,433.92,434.7],'ITU 5.138 Region 1'),
 ('ISM-915MHz',902,928,[902.1,915,927.9],'ITU 5.150 Region 2 supplement'),
 ('ISM-2p4GHz',2400,2500,[2400.1,2450,2499.9],'ITU 5.150'),
 ('ISM-5p8GHz',5725,5875,[5725.1,5800,5874.9],'ITU 5.150')]
points={30000000:'endpoint'}
for mhz in range(50,6000,50):points[mhz*1000000]='base-grid'
points[5999999000]='upper-endpoint-below-exclusive-limit'
for mhz in [2384.9,2385,2385.1,2494.9,2495,2495.1]:points[round(mhz*1e6)]='RF-path-boundary'
for name,lo,hi,anchors,scope in bands:
 for mhz in anchors:points.setdefault(round(mhz*1e6),'band-anchor')
rows=[]
for i,(hz,reason) in enumerate(sorted(points.items()),1):
 rows.append(dict(point=i,frequency_hz=hz,reason=reason,bands=[n for n,lo,hi,a,s in bands if lo*1e6<=hz<=hi*1e6]))
coverage=[dict(name=n,low_hz=round(lo*1e6),high_hz=round(hi*1e6),scope=s,points=[r['frequency_hz'] for r in rows if n in r['bands']]) for n,lo,hi,a,s in bands]
assert all(len(b['points'])>=3 for b in coverage)
plan=dict(points=rows,band_coverage=coverage,base_step_hz=50000000,requested_upper_hz=6000000000,upper_limit_exclusive=True,rx_sample_rate_hz=1000000,tx_sample_rate_hz=1000000,tx_power_dbm=0,settle_seconds=2,measure_seconds=5,sources=['https://www.itu.int/net/ITU-R/terrestrial/faq/index.html','https://rsgb.services/public/bandplans/'],note='Band labels guide laboratory coverage, not national operating permissions. Region 1 emphasis plus 222/915 MHz Region 2 supplements; 868 MHz lies on the base grid at 850/900, so explicit SRD anchors are added separately only if requested.')
# Include the European 868 MHz SRD range as useful extra coverage; it is not an ITU ISM band.
for hz in [863100000,868100000,869900000]:
 if hz not in points:plan['points'].append(dict(frequency_hz=hz,reason='European-SRD-supplement',bands=['SRD-863-870MHz']))
plan['points'].sort(key=lambda r:r['frequency_hz'])
for i,r in enumerate(plan['points'],1):r['point']=i
plan['note']='Band labels guide laboratory coverage, not national operating permissions. Region 1 emphasis, supplementary Region 2 amateur/ISM coverage and three European 863–870 MHz SRD points (not labeled ITU ISM).'
Path(__file__).with_name('plan.json').write_text(json.dumps(plan,indent=2)+'\n')
print(len(plan['points']),'frequencies; all',len(coverage),'amateur/ISM ranges have >=3 points')
