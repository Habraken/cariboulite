#include <SoapySDR/Device.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
int main(int argc,char **argv) {
 try {
  if(argc!=2) throw std::runtime_error("module path required");
  auto error=SoapySDR::loadModule(argv[1]); if(!error.empty())throw std::runtime_error(error);
  auto found=SoapySDR::Device::enumerate("driver=Cariboulite,channel=HiF");
  if(found.size()!=1)throw std::runtime_error("expected one HiF device");
  std::unique_ptr<SoapySDR::Device,void(*)(SoapySDR::Device*)> dev(SoapySDR::Device::make(found[0]),SoapySDR::Device::unmake);
  if(!dev)throw std::runtime_error("device make failed");
  dev->setFrequency(SOAPY_SDR_RX,0,430100000.);
  dev->setGainMode(SOAPY_SDR_RX,0,true);
  dev->setBandwidth(SOAPY_SDR_RX,0,160000.);
  auto *stream=dev->setupStream(SOAPY_SDR_RX,SOAPY_SDR_CF32);
  std::vector<float> buf(2*16384);void *buffs[]={buf.data()};
  auto rates=dev->listSampleRates(SOAPY_SDR_RX,0);bool passed=true;
  for(double rate:rates) {
   dev->setSampleRate(SOAPY_SDR_RX,0,rate);
   double reported=dev->getSampleRate(SOAPY_SDR_RX,0);
   if(dev->activateStream(stream)!=0)throw std::runtime_error("activate failed");
   auto begin=std::chrono::steady_clock::now();
   auto read=[&](){int flags=0;long long t=0;return dev->readStream(stream,buffs,16384,flags,t,100000);};
   while(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<.3)read();
   begin=std::chrono::steady_clock::now();size_t samples=0;unsigned errors=0,timeouts=0;
   while(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<1.5) {
    int n=read();if(n>0)samples+=n;else if(n==SOAPY_SDR_TIMEOUT)++timeouts;else ++errors;
   }
   double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
   if(dev->deactivateStream(stream)!=0)throw std::runtime_error("deactivate failed");
   double observed=samples/elapsed;
   bool ok=std::fabs(reported-rate)<1 && std::fabs(observed/rate-1)<.05 && !errors;
   passed &= ok;
   std::cout<<"RATE_RESULT {\"requested_hz\":"<<rate<<",\"reported_hz\":"<<reported<<",\"samples\":"<<samples<<",\"elapsed_seconds\":"<<elapsed<<",\"observed_samples_per_second\":"<<observed<<",\"errors\":"<<errors<<",\"timeouts\":"<<timeouts<<",\"passed\":"<<(ok?"true":"false")<<"}"<<std::endl;
  }
  dev->closeStream(stream);
  std::cout<<"RATE_CHECK_COMPLETE passed="<<passed<<std::endl;
  return passed?0:2;
 }catch(const std::exception &e){std::cerr<<"CHECK_FAILED "<<e.what()<<std::endl;return 1;}
}
