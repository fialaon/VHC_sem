load("linear")
load("nonlinear")

% graphs
FontSize = 14;
figure
set(gcf,'Color','white');
hold all
plot(t(801:1601), y_ref(801:1601),'-black')
plot(t(801:1601), y1(801:1601),'-red')
plot(t(801:1601), y2(801:1601),'-blue')
grid on
xlabel('Time [s]')
ylabel('Lateral position [m]')
set(gca,'FontSize',FontSize)
legend('Reference','Linear MPC','Nonlinear MPC')

figure
set(gcf,'Color','white');
hold all
plot(t(801:1601), (180/pi)*psi_ref(801:1601),'-black')
plot(t(801:1601), (180/pi)*psi1(801:1601),'-red')
plot(t(801:1601), (180/pi)*psi2(801:1601),'-blue')
grid on
xlabel('Time [s]')
ylabel('Yaw [deg]')
set(gca,'FontSize',FontSize)
legend('Reference','Linear MPC','Nonlinear MPC')

figure
set(gcf,'Color','white');
hold all
plot(t(801:1601), (180/pi)*psi_dot_ref(801:1601),'-black')
plot(t(801:1601), (180/pi)*psi_dot1(801:1601),'-red')
plot(t(801:1601), (180/pi)*psi_dot2(801:1601),'-blue')
grid on
xlabel('Time [s]')
ylabel('Yaw rate [deg/s]')
set(gca,'FontSize',FontSize)
legend('Reference','Linear MPC','Nonlinear MPC')

figure
set(gcf,'Color','white');
hold all
plot(t(801:1601), RWA1(801:1601),'-red')
plot(t(801:1601), RWA2(801:1601),'-blue')
grid on
xlabel('Time [s]')
ylabel('Road wheel angle [deg/s]')
set(gca,'FontSize',FontSize)
legend('Linear MPC','Nonlinear MPC')

figure
set(gcf,'Color','white');
hold all
plot(t(801:1601), (180/pi)*dRWA1(801:1601),'-red')
plot(t(801:1601), (180/pi)*dRWA2(801:1601),'-blue')
grid on
xlabel('Time [s]')
ylabel('Road wheel angle rate [deg/s]')
set(gca,'FontSize',FontSize)
legend('Linear MPC','Nonlinear MPC')