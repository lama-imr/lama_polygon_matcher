#!/usr/bin/env perl

if (@ARGV < 4) {
    print STDERR "usage: $0 <file.results> <datasetdir> <isResultsSymmetric(0/1)><prefix>\n";
    print STDERR "file.results          contains results in form 'battle-1 rat-14 X Y', where X is dissimilarity and Y is processing time\n";
    print STDERR "datasetdir            pointer to directory with images\n";
    print STDERR "isResultsSymmetric    1 .. file.results contains both 'battle-1 rat-4' and 'rat-14 battle-2'\n";
    print STDERR "                      0 .. file.results contains only one of them\n";
    exit;
}

my $imWidth = 80; #width of thumbnails of tested images

my $minTh = 0;    #min threshold for distance for creating PR-graph
my $maxTh = 0.3;  #max threshold for PR-graph
my $deltaTh = 0.02; #step for creating PR-graph
my $maxprint = 12;  #number of most similar images listed in the table



my ($in, $datasetdir, $symmetric, $prefix) = @ARGV;

open(my $fi,"<$in") or die "Cannot open $in: $!\n";
print STDERR "Loading from $in..";
my @data = <$fi>;
close $fi;
chomp @data;
print " loaded " . scalar(@data) . " results\n";


print STDERR "Creating main hash.. ";
my %g= ();
my %family = (); #number of images in each family (e.g. beetle, device0, horse..)
my $minDist, $maxDist;

my $first = 1;
foreach $line (@data) {
    $line =~ s/^\s+//g;
    my ($name1, $name2, $dist, $time) = split(/\s+/,$line);
    my ($n1, $idx1) = split('-',$name1);
    my ($n2, $idx2) = split('-',$name2);
    push @{ $h{$n1}{$n2} }, "$dist $time";
    $g{$name1}{$name2} = "$dist $time";

    if ($symmetric == 0) {
        push @{ $h{$n2}{$n1} }, "$dist $time";
        $g{$name2}{$name1} = "$dist $time";
    }

    $family{$n1} = max($family{$n1},$idx1);
    if ($dist < $minDist or $first == 1) {
        $minDist = $dist;
    }
    if ($dist > $maxDist or $first == 1) {
        $maxDist = $dist;
    }   
    $first = 0;
}
print STDERR "main hash created\n";
print STDERR "Computing PR graph\n";

my $numClasses = scalar(keys %family);
print "MinDist=$minDist, maxDist=$maxDist ";

#create pr-graph
open(my $fo,">$prefix.pr.dat");
print $fo "#threashold avgPrecision avgRecall\n";
my $prc = 0;
$maxTh = $maxDist;
$deltaTh = $maxTh / 20;
print STDERR "maxTh=$maxTh, deltaTh=$deltaTh\n";
for(my $th = $minTh; $th < $maxTh; $th+=$deltaTh) {
    print "$th ";
    my $meanPrecision = 0;
    my $meanRecall = 0;
    my $count=0;
    foreach my $k1 (keys %g) {
        my $relevant = 0;
        my $allreturned = 0;
        my $fam1 = getFamiliyName($k1);
        foreach my $k2 (keys %{$g{$k1}}) {
            my ($sim,$time) = split(/\s+/,$g{$k1}{$k2});
            if ($sim <= $th) {
                my $fam2 = getFamiliyName($k2);
#                print "Family of $k2 is $fam\n";
                if ($fam1 eq $fam2) {
                    $relevant++;
                }
                $allreturned++;
            }
        }
        if ($allreturned > 0) {
            my $precision = $relevant / $allreturned;
            my $recall = $relevant / $family{$fam1};
#        print STDERR "th=$th AllReturned $allreturned, relevant=$relevant: pr=$precision, recall=$recall\n";
            $meanPrecision += $precision;
            $meanRecall += $recall;
            $count++;
        }
    }
    if ($count > 0) {
        $meanPrecision /= $count;
        $meanRecall /= $count;  
        print $fo "$th $meanPrecision $meanRecall\n";
    } else {
        print $fo "#th=$th, no result where obtained under this threshold\n";
    }
    $prc++;
    print STDERR "Finished: " . sprintf("%.2lf \%\r",100*$prc/(($maxTh-$minTh)/$deltaTh));
}
close $fo;
print "\n";

open(my $fo,">$prefix.pr.gpl");
print $fo "set term png size 1536,1024 truecolor\n";
print $fo "set output '$prefix.pr.png'\n";
print $fo "set title 'Precision vs Recall for $in'\n";
print $fo "set xlabel 'recall'\n";
print $fo "set ylabel 'precision'\n";
print $fo "set grid\n";
#print $fo "plot '$prefix.pr.dat' u 3:2 w l notitle\n";
print $fo "plot '$prefix.pr.dat' u 3:2 w l notitle, '' u 3:2:1 w labels\n";
print $fo "set output\n";
close $fo;
system "gnuplot $prefix.pr.gpl";


print STDERR "Number of images in each family:\n";
my $maximumScore = 0;
foreach my $k (sort keys %family) {
    print STDERR "$k $family{$k} ";
    my $size = $family{$k};
    $maximumScore += $size*$size;
}
print STDERR "\nMaximum score for BUE: $maximumScore\n";
print STDERR "Computing BullEye score:\n";
my $score = 0;
$prc = 0;
foreach my $k1 (sort keys %g) {
    my @sims= ();
    my $k1fam = getFamiliyName($k1);
    my $instacesSize = $family{$k1fam};

    foreach my $k2 (keys %{$g{$k1}}) {
        my ($sim,$time) = split(/\s+/,$g{$k1}{$k2});
        push @sims, "$sim $time $k2";
    }
    my @a = sort sortByDist @sims;
    if (@a < 2*$instacesSize) {
        print STDERR "Example $k1, familyName=$k1fam, max.instances=$instacesSize\n";
        print STDERR "Cannot compute BUE for $k1:, we have only " . @a . " matching shapes!\n";
    }
    for(my $i=0;$i<2*$instacesSize;$i++) {
        my ($sim1, $time1, $name1) = split(/\s+/,$a[$i]);
        if (getFamiliyName($k1) eq getFamiliyName($name1)) {
            $score++;
        }
    }
    $prc++;
    print STDERR "Finished: " . sprintf("%.2lf \%\r",100*$prc/(scalar(keys %g)));
}
#my $maximumScore = 28000;
my $bue = 100*$score / $maximumScore;
print STDERR "Score is $score out of $maximumScore = $bue %\n";

open(my $fob,">$prefix.bue");
print $fob "#bull-eye\n";
print $fob "$bue\n";
close $fob;

die;



print STDERR "Creating html: $prefix.html .. ";
open(my $fo,">$prefix.html");
print $fo "<html><head></head><body>\n";
print $fo "<h3>Bull Eye: $bue % = $score out of $maximumScore </h3>\n";
print $fo "<b>".scalar(keys %family)." classes:</b>\n";
foreach my $k (keys %family) {
    print $fo "$k($family{$k}),";
}
print $fo "<br/>\n";
my $bf = "<font color=\"blue\">";
my $ebf = "</font>";
my $rf = "<font color=\"red\">";
my $erf = "</font>";

print $fo "Precision-recall for distances in interval: $bf $minTh $ebf : $bf $maxTh $ebf : $bf $deltaTh $ebf \n";
print $fo "Dissimilarity: min= $bf $minDist $ebf, max= $bf $maxDist $ebf <br/>\n";
if ($minTh > $minDist or $maxTh < $maxDist) {
    print $fo "<b>$rf You have to rerun processData.pl and set minTh=$minDist and maxTh=$maxDist! $erf </b><br/>\n";
    print STDERR "You have to rerun processData.pl and set minTh=$minDist and maxTh=$maxDist! \n";
} else {
    print $fo "Limit for PR graph are ok<br/>\n";
}

print $fo "<center><a href=\"$prefix.pr.png\"><img src=\"$prefix.pr.png\" width=\"800\"/></a></center>\n";
print $fo "<table border=\"1\">\n";
#select most n-similar
print $fo "<tr bgcol=\"#CCFFFF\"><td align=\"center\"><b>Query</b></td>";
for(my $i=0;$i<$maxprint;$i++) {
    print $fo "<td align=\"center\"><b>$i-th</b></td>";
}
print $fo "</tr>\n";

foreach my $k1 (sort keys %g) {
    my $image1=getImage($k1);
    my $k1fam = getFamiliyName($k1);
    my $instacesSize = $family{$k1fam};
    print $fo "<tr>\n<td bgcolor=\"#CCFF99\" align=\"center\"><img width=\"$imWidth\" src=\"$image1\"/><br/>\n";
    print $fo "$k1, fs=$family{$k1fam}</td>\n";
    my @sims= ();
    foreach my $k2 (keys %{$g{$k1}}) {
        my ($sim,$time) = split(/\s+/,$g{$k1}{$k2});
        push @sims, "$sim $time $k2";
    }
#    print "Best $k1:\n";
    my @a = sort sortByDist @sims;
    for(my $i=0;$i<( @a<$maxprint?@a:$maxprint);$i++) {
        my ($sim1, $time1, $name1) = split(/\s+/,$a[$i]);
        my $image2=getImage($name1);
        my $bgcol = "#CC0000";
        if (getFamiliyName($k1) eq getFamiliyName($name1)) {
            $bgcol="#00DD00";
        }
        print $fo "<td align=\"center\" bgcolor=\"$bgcol\"><img width=\"$imWidth\" src=\"$image2\"/><br/><font color=\"blue\">$sim1</font> $name1</td>\n";
#       print "$i $name1 $sim1\n";
    }
    print $fo "</tr>\n";
#   print "\n";
}
print $fo "</table>\n";
print $fo "</body></html>\n";
close $fo;

print STDERR "ok\n";
exit;









#-------------------------------------------------
sub getFamiliyName {
    my ($n) = @_;
    if ($n !~ m/-/) {
        return $n;
    } else {
        my ($fn, $idx) = split('-',$n);
        return $fn;
    }
}





#-------------------------------------------------
sub getImage {
    my ($key) = @_;

    if (-e "$datasetdir/$key.txt.png") {
        return "$datasetdir/$key.txt.png";
    }

    if (-e "$datasetdir/$key.jpg") {
        return "$datasetdir/$key.jpg";
    }

    my @names = <$datasetdir/$key-*.txt.png>;
    chomp @names;
    if (@names == 0) {
        print STDERR "cannot find images for key name '$key'\n";
#        die;
        return "/home/vojta/jezek.jpg";
    }
    return $names[0];
}

#-------------------------------------------------
sub sortByDist {
    my ($sim1, $time1, $name1) = split(/\s+/,$a);
    my ($sim2, $time2, $name2) = split(/\s+/,$b);
#    print "SBD: $a, $b: $sim1 $sim2\n";
    if ($sim1 < $sim2) { return -1; }
    elsif ($sim1 == $sim2) { return 0; }
    else { return 1; }
}

#-------------------------------------------------
sub getStat {
    my ($ref, $col) = @_;
    my $s = 0;
    foreach my $val (@{ $ref }) {
        my @b = split(/\s+/,$val);
        $s += $b[$col];
    }
    $s /= @{ $ref };
    return $s;
}  

#-------------------------------------------------
sub max {
    my ($a, $b) = @_;
    if ($a > $b) { 
        return $a;
    } else {
        return $b;
    }
}



