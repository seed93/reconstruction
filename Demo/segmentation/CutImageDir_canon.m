function CutImageDir_canon(dir_name)
    mask_dir = [dir_name '/mask/'];
    result_dir = [dir_name '/result/'];
    mkdir(mask_dir);
    mkdir(result_dir);
    isrotated = ones(10,1);
    for j=1:10
        fprintf('cam %d start\n', j);
        bk_img = imread(sprintf('%s//0002_Cam%d.jpg', dir_name, j-1));
%         if isrotated(j)
%             bk_img = rot90(bk_img,1);
%             imwrite(bk_img, sprintf('%s//0002_Cam%d.jpg', dir_name, j-1));
%         end
        bk_img = imresize(bk_img,[NaN 350]);
        bk_img = im2double(bk_img);
        mean_b = mean(bk_img,3)-eps;
        bk_img2 = bsxfun(@minus, bk_img, mean_b);
        parfor i=15:20
            CutOneImage(sprintf('%s//%.4d_Cam%d.jpg', dir_name, i, j-1), ...
               bk_img2, mean_b, sprintf('%s//%.4d_Cam%d.jpg', mask_dir, i, j-1), ...
               sprintf('%s//%.4d_Cam%d.jpg', result_dir, i, j-1), isrotated(j)); 
        end
    end
end

function CutOneImage(img1_name,img2_,mean_img2,mask_name,temp_name,isrotated)
    img1 = imread(img1_name);
    if isrotated
        img1 = rot90(img1,1);
        imwrite(img1, img1_name, 'Quality', 80);
    end
    img1 = imresize(img1,[NaN 350]);
    img1 = im2double(img1);
    [h,w,~] = size(img1);
    a = bsxfun(@minus, img1, mean_img2);
    S = sum(a.*img2_, 3)./sqrt(sum(a.*a,3).*sum(img2_.*img2_,3));
    mask=im2bw(1-S,0.9);
    mask=logical(imfill(double(mask)));
    mask=imopen(mask,strel('disk',3));
    [I,J] = ind2sub(size((mask)),find(mask>0));
    x = round(sum(I)/length(I));
    y = round(sum(J)/length(J));
    [L,~]=bwlabeln(mask,4);
    mask = zeros(h,w);
    mask(L==L(x,y)) = 1;
    mask = logical(mask);
    img1 = rgb2gray(img1);
    img1(~mask) = 0;
    imwrite(img1, temp_name);
    mask = imresize(mask,[NaN 3456]);
    mask = logical(mask);
    imwrite(mask, mask_name);
end