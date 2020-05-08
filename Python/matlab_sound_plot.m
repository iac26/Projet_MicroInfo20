hold on
grid on
save = 0;
save = save900;
plot(save.(1)) % fb
plot(save.(2)) % lr
plot(atan2(save.(1), save.(2)))
hold off
xlabel("numero de sample")
ylabel("angle [rad]")
ylim([-3.3 3.3])
xlim([0 450])
legend("phase gauche-droite", "phase avant-arriere", "angle du son", 'Location','southwest')

set(gca,'FontSize',14)
set(gcf,'position',[20,20,1000,400])

exportgraphics(gcf,'son_dir.png','Resolution',300)