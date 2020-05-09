
grid on
f = fit(dist.(2), dist.(1),'b*x^-0.5')
hold on
plot(dist.(2), dist.(1), '*') 
plot(f)
hold off
xlabel("numero de sample")
ylabel("angle [rad]")
xlim([0 2600])
ylim([0 110])

set(gca,'FontSize',14)
set(gcf,'position',[20,20,1000,400])

exportgraphics(gcf,'dist.png','Resolution',300)