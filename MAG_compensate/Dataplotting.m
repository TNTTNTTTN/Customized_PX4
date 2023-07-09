msg = ulogreader('log1.ulg');
thrust = readTopicMsgs(msg, 'TopicNames', {'actuator_motors'});
mag = readTopicMsgs(msg, 'TopicNames', {'sensor_mag'});
basethrust = thrust.TopicMessages{1,1}.control(:,1);
mag0_x = mag.TopicMessages{1,1}.x;
mag0_y = mag.TopicMessages{1,1}.y;
mag0_z = mag.TopicMessages{1,1}.z;
mag1_x = mag.TopicMessages{2,1}.x;
mag1_y = mag.TopicMessages{2,1}.y;
mag1_z = mag.TopicMessages{2,1}.z;

subplot(2,3,1)
md = fitlm(basethrust.^2, mag0_x, 'linear')
plot(md)
title("Thrust vs mag0-x");
xlabel("Thrust");
ylabel("Mag0-x[Gauss]");

subplot(2,3,2)
md = fitlm(basethrust.^2, mag0_y, 'linear')
plot(md)
title("Thrust vs mag0-y");
xlabel("Thrust");
ylabel("Mag0-y[Gauss]");
hold off

subplot(2,3,3)
md = fitlm(basethrust.^2, mag0_z, 'linear')
plot(md)
title("Thrust vs mag0-z");
xlabel("Thrust");
ylabel("Mag0-z[Gauss]");

subplot(2,3,4)
md = fitlm(basethrust.^2, mag1_x, 'linear')
plot(md)
title("Thrust vs mag1-x");
xlabel("Thrust");
ylabel("Mag1-x[Gauss]");

subplot(2,3,5)
md = fitlm(basethrust.^2, mag1_y, 'linear')
plot(md)
title("Thrust vs mag1-y");
xlabel("Thrust");
ylabel("Mag1-y[Gauss]");

subplot(2,3,6)
md = fitlm(basethrust.^2, mag1_z, 'linear')
plot(md)
title("Thrust vs mag1-z");
xlabel("Thrust");
ylabel("Mag1-z[Gauss]");