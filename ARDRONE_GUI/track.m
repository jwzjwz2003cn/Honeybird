function data = track(KR)
frames = 150;
data = zeros(frames, 10);
elapsedTime = 0;

%for j=1:frames
while(1)
    disp('-----');
    tic;
    [posD, posB] = TrackFrameBBOX(KR)
    %elapsedTime = elapsedTime + toc
    toc;
    %data(j,1) = elapsedTime;
    %if( size(posD) ~= 0 )
    %    data(j,2:4) = transpose(posD);
    %end
    % if( size(posB) ~= 0 )
    %    [m,n] = size(transpose(posB));
    %    data(j,5:n) = transpose(posB);
    %end
end

%figure;
%subplot(3,1,1); plot( data(1:frames,1),data(1:frames,2) );
%subplot(3,1,2); plot( data(1:frames,1),data(1:frames,3) );
%subplot(3,1,3); plot( data(1:frames,1),data(1:frames,4) );
%figure;
%scatter3( data(1:frames,2), data(1:frames,3), data(1:frames,4) );
%xlabel('x')
%ylabel('y')
