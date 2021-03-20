source=5*sin(2*3*t);
noise=5*sin(2*50*3*t);
reference_noise=5*sin(2*50*3*t+ 3/20);
cancelled = LMS_MCU(source, noise, reference_noise);