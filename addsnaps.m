function snaptot =  addsnaps(snap1,path2)
%     snap1 = imread(path1);
    snap2 = imread(path2);
    snaptot = uint8(ones(size(snap1,1),size(snap1,2),size(snap1,3))*255);
    for i = 1 : size(snap1,1)
        for j = 1 : size(snap1,2)
            if snap1(i,j,1) < 255 || snap1(i,j,2) < 255 || snap1(i,j,3) < 255
                snaptot(i,j,:) = snap1(i,j,:);
            end
            if snap2(i,j,1) < 255 || snap2(i,j,2) < 255 || snap2(i,j,3) < 255
                snaptot(i,j,:) = snap2(i,j,:);
            end
        end
    end
%     imshow(snaptot);
end
