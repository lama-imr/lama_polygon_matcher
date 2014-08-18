#!/usr/bin/env perl

if (@ARGV < 3) {
    print STDERR "usage: $0 <dataset> <input-prefix> <output-prefix>\n";
    exit;
}
my ($dir, $in, $out) = @ARGV;

open(my $fo,">$out.allbue");
print $fo "#moments bue\n";
$|=1;

foreach my $c (5..40) {
    if (-e "$in.$c.fullresults") {
        system "./processResults.pl $in.$c.fullresults $dir 1 $out.$c";
        open(my $fi,"<$out.$c.bue");
        my @a = <$fi>;
        close $fi;
        chomp @a;
        my $r = $a[1];
        print $fo "$c $r\n";
    }
}
close $fo;
