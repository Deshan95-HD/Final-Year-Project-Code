 clear;
rpi = raspi('192.168.1.102','pi','raspberry');
mycam = webcam(rpi);
tic;
for ii = 1:50
img = snapshot(mycam);
    imagesc(img)
    drawnow
end
toc;