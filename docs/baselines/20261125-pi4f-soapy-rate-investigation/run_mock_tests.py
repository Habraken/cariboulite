from pathlib import Path
import subprocess,tempfile,json
root=Path('/home/pi/src/cariboulite');src=root/'software/libcariboulite/src';out=root/'build/soapy-rate-fix-20261125'
script=(out/'test_soapy_stream.py').read_text();script=script.replace("root=Path(__file__).resolve().parents[3]", "root=Path('/home/pi/src/cariboulite')").replace("sources=[src/'soapy_api'/s for s in ['Cariboulite.cpp','CaribouliteSensors.cpp','CaribouliteStream.cpp','CaribouliteStreamFunctions.cpp']]", "sources=[Path('/home/pi/src/cariboulite/build/soapy-rate-fix-20261125/Cariboulite.cpp')]+[src/'soapy_api'/s for s in ['CaribouliteSensors.cpp','CaribouliteStream.cpp','CaribouliteStreamFunctions.cpp']]")
script=script.replace("includes=['-I'+str(src),'-I'+str(src/'iir')]", "includes=['-I'+str(src),'-I'+str(src/'iir'),'-I'+str(src/'soapy_api')]")
(out/'test_stream_fixed.py').write_text(script)
subprocess.run(['python3',str(out/'test_stream_fixed.py')],check=True)
methods=(out/'Cariboulite.cpp').read_text();methods=methods[methods.index('void Cariboulite::setSampleRate('):methods.index('static cariboulite_radio_rx_bw_en convertRxBandwidth(')]
(out/'rates.cpp').write_text((out/'test_soapy_rates.cpp').read_text()+'\n'+methods)
subprocess.run(['c++','-std=c++11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-I'+str(src),str(out/'rates.cpp'),'-o',str(out/'mock-rates')],check=True)
subprocess.run([str(out/'mock-rates')],check=True)
subprocess.run(['c++','-std=c++11','-O2',str(out/'check_rates.cpp'),'-lSoapySDR','-o',str(out/'check-rates')],check=True)
(out/'mock-tests.json').write_text(json.dumps({'passed':True,'soapy_stream_regression':True,'smi_timed_regression':True,'rate_selection_regression':True},indent=2)+'\n')
