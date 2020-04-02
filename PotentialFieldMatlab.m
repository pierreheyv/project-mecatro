xtarget = 0;
ytarget = 1.3;

xpos= 2;
ypos= 1.3;

xadv=2.5;
yadv=1.5;
distlimadv=0.5;
krepadv=0.03;

distlim=0.25;
katt=7;
krep=0.003;

grid=0.05;
X= 0:grid:3;
Y= 0:grid:2;
sizeX=size(X);
sizex=sizeX(2);
sizeY=size(Y);
sizey=sizeY(2);
[xx,yy]=meshgrid(X,Y);

obstacle1=zeros(2,sizey); %mur gauche
obstacle2=3*ones(2,sizey); %mur droite
for j=1:sizey
    obstacle1(2,j)=grid*(j-1);
    obstacle2(2,j)=grid*(j-1);
end
obstacle3=zeros(2,sizex); %mur bas
obstacle4=2*ones(2,sizex); %mur haut
for j=1:sizex
    obstacle3(1,j)=grid*(j-1);
    obstacle4(1,j)=grid*(j-1);
end

distobstacle=300*ones(sizex,sizey); % distance de l'obstacle le plus proche
U=zeros(sizey,sizex);
for i=1:sizex
    for j=1:sizey
        %distance obstacle1
        for k=1:sizey
            dist=sqrt((X(i)-obstacle1(1,k))^2+(Y(j)-obstacle1(2,k))^2);
            if (dist<distobstacle(i,j))
                distobstacle(i,j)=dist;
            end
        end
        %distance obstacle2
        for k=1:sizey
            dist=sqrt((X(i)-obstacle2(1,k))^2+(Y(j)-obstacle2(2,k))^2);
            if (dist<distobstacle(i,j))
                distobstacle(i,j)=dist;
            end
        end
        %distance obstacle3
        for k=1:sizex
            dist=sqrt((X(i)-obstacle3(1,k))^2+(Y(j)-obstacle3(2,k))^2);
            if (dist<distobstacle(i,j))
                distobstacle(i,j)=dist;
            end
        end
        %distance obstacle4
        for k=1:sizex
            dist=sqrt((X(i)-obstacle4(1,k))^2+(Y(j)-obstacle4(2,k))^2);
            if (dist<distobstacle(i,j))
                distobstacle(i,j)=dist;
            end
        end
        
        if (distobstacle(i,j)<10^-5)
            distobstacle(i,j)=0.03;
        end
        
        distadv=sqrt((X(i)-xadv)^2+(Y(j)-yadv)^2);
        if (distadv<10^-5)
            distadv=0.03;
        end
        
        disttarget2=(X(i)-xtarget).^2+(Y(j)-ytarget).^2;
        U(j,i)=katt*disttarget2/2;
        if (distobstacle(i,j)<distlim) 
            U(j,i)=U(j,i)+krepadv*((1/distobstacle(i,j) - 1/distlim)^2)/2;
            %U(j,i)=U(j,i)+krep*(1/(distobstacle(i,j) - distlim))/2;
        end
        if (distadv<distlimadv) 
            U(j,i)=U(j,i)+krepadv*((1/distadv - 1/distlimadv)^2)/2;
            %U(j,i)=U(j,i)+krep*(1/(distobstacle(i,j) - distlim))/2;
        end
    end
end

figure;

stem(xadv,yadv,'LineStyle','none','MarkerSize',10,'color','r','MarkerFaceColor','r');
hold on
contour(X,Y,U);
hold on

U=-U;
[px,py] = gradient(U,grid,grid);
quiver(X,Y,px,py)
x=xpos/grid +1;
y=ypos/grid +1;
Fx=px(y,x)
Fy=py(y,x)
